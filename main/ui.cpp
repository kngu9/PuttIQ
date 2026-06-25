// ui.cpp — on-device LVGL pipeline + display/touch bring-up + shared UI data.
//
// Extracted from main.ino as a behavior-preserving MOVE. This owns the panel
// (TFT_eSPI `tft`), the touch driver (chsc6x), the LVGL display/indev/tick
// callbacks, the deferred screen-swap pipeline, and the UI data globals. The
// app<->UI policy (ui_on_click, ui_sync_from_state, g_inConfig) stays in
// main.ino and talks to this through ui.h.
//
// DEVICE-ONLY: lv_xiao_round_screen.h defines `tft`/`chsc6x_*`/`xiao_disp_init`,
// so this is the single translation unit that may include it. The host builds
// (`make test`, `make preview`) do NOT compile this file.
#include <Arduino.h>
#include <Wire.h>
#define USE_TFT_ESPI_LIBRARY
#include <lv_xiao_round_screen.h>   // tft, chsc6x_*, xiao_disp_init, screen_rotation (+ lvgl, TFT_eSPI)
#include "ui.h"                      // ui_* API + shared globals + UiScreen
#include "imu_types.h"               // TRACE_CAP (trace backing-array size)

// Mirror of main.ino's old display gate (always true on this hardware).
static const bool USE_ROUND_DISPLAY = true;

// ===========================================================================
// LVGL v9.5 on-device pipeline state (file-private).
// ===========================================================================
static bool          g_lvglReady = false;
static lv_display_t* g_disp      = nullptr;

// Partial render buffer: 240 x 40 RGB565 = 19200 px = 38400 bytes.
// If RAM overflows, shrink the row count (e.g. 240*24). See compile report.
static lv_color_t    g_lvbuf[240 * 40];

// ===========================================================================
// Shared UI data (declared extern in ui.h).
// ===========================================================================
UiScreen g_uiCur          = UI_NONE;
int      g_countdownSecs  = 5;     // digit currently shown on UI_COUNTDOWN
int      g_countdownTotal = 5;     // = MANUAL_COUNTDOWN_MS / 1000 (5000 / 1000)
UiResult g_uiResult;               // populated from the app on an accepted putt
float    g_traceX[TRACE_CAP];      // backing storage for g_uiResult.traceX/Y
float    g_traceY[TRACE_CAP];
// Auto/listening face by default (boot mode is MODE_AUTO). The app keeps this in
// sync before any home rebuild; see ui.h.
bool     g_uiHomeAuto = true;

// ---------------------------------------------------------------------------
// Safe deferred screen switching.
//
// THE #1 RULE: event callbacks NEVER build/load/delete LVGL screens. They only
// set flags (g_uiReq, ...). ui_apply() runs in loop() AFTER lv_timer_handler()
// and does the actual build -> lv_screen_load -> delete-old. Deleting a screen
// mid-event would free objects still being walked by LVGL's dispatcher and
// freeze/crash the UI.
// ---------------------------------------------------------------------------
static UiScreen g_uiReq   = UI_NONE;   // pending request (app uses ui_request())
static bool     g_uiForce = false;     // rebuild even if requested == current

// ===========================================================================
// LVGL callbacks.
// ===========================================================================
// LVGL tick source.
static uint32_t lv_millis_cb(void) { return millis(); }

// Push a rendered area to the GC9A01 via TFT_eSPI.
// NOTE: the `true` arg to pushColors() = byte-swap. LVGL emits little-endian
// RGB565; if colors look wrong on the panel, toggle this flag FIRST.
static void lv_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)px_map, w * h, true);
  tft.endWrite();
  lv_display_flush_ready(disp);
}

// Touch read (mirrors chsc6x_read in lv_xiao_round_screen.h, v9 signature).
static void lv_touch_read_cb(lv_indev_t* indev, lv_indev_data_t* data) {
  (void)indev;
  if (!chsc6x_is_pressed()) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }
  lv_coord_t tx = 0, ty = 0;
  chsc6x_get_xy(&tx, &ty);
  data->point.x = tx;
  data->point.y = ty;
  data->state = LV_INDEV_STATE_PRESSED;
}

// ===========================================================================
// Touch bring-up.
// ===========================================================================
static void initTouchButton() {
#if defined(TOUCH_INT)
  pinMode(TOUCH_INT, INPUT_PULLUP);
#endif
  Wire.begin();
}

// ===========================================================================
// Public API.
// ===========================================================================
bool ui_ready() { return g_lvglReady; }

void ui_request(UiScreen s, bool force) {
  g_uiReq = s;
  if (force) {
    g_uiForce = true;
  }
}

// Build a fresh screen object for `s` and populate it.
static lv_obj_t* ui_build_screen(UiScreen s) {
  lv_obj_t* scr = lv_obj_create(NULL);
  // Screens are static instrument faces — never scroll/bounce (LVGL default on).
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
  switch (s) {
    case UI_HOME:
      // Show the "listening" (auto-style) face whenever we're armed and waiting
      // for a stroke (auto home, OR manual after the countdown). Show the manual
      // START face only at the manual idle home. g_uiHomeAuto carries the old
      // `state != SENSOR_HOME` decision (kept current by the app).
      ui_build_home(scr, g_uiHomeAuto);
      break;
    case UI_COUNTDOWN:
      ui_build_countdown(scr, g_countdownSecs, g_countdownTotal);
      break;
    case UI_RESULT:
      ui_build_result(scr, g_uiResult);
      break;
    case UI_DETAILS:
      ui_build_details(scr, g_uiResult);
      break;
    case UI_CONFIG:
      ui_build_config(scr);
      break;
    case UI_NOIMU: {
      lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
      lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
      lv_obj_t* l = lv_label_create(scr);
      lv_label_set_text(l, "NO IMU");
      lv_obj_set_style_text_color(l, lv_color_hex(0xFFB000), LV_PART_MAIN);
      lv_obj_set_style_text_font(l, &lv_font_montserrat_28, LV_PART_MAIN);
      lv_obj_center(l);
      break;
    }
    case UI_SETTLING:
    default: {
      lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
      lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
      lv_obj_t* l = lv_label_create(scr);
      lv_label_set_text(l, "...");
      lv_obj_set_style_text_color(l, lv_color_hex(0x888888), LV_PART_MAIN);
      lv_obj_set_style_text_font(l, &lv_font_montserrat_28, LV_PART_MAIN);
      lv_obj_center(l);
      break;
    }
  }
  return scr;
}

// Apply a pending screen request: build new, load it, delete the OLD active
// screen. SAFE because this runs in loop(), never inside an event callback.
void ui_apply(void) {
  if (g_uiReq == UI_NONE) {
    return;
  }
  if (g_uiReq == g_uiCur && !g_uiForce) {
    return;
  }
  g_uiForce = false;

  lv_obj_t* old = lv_screen_active();
  lv_obj_t* fresh = ui_build_screen(g_uiReq);
  lv_screen_load(fresh);
  if (old && old != fresh) {
    lv_obj_delete(old);  // safe outside events
  }
  g_uiCur = g_uiReq;
}

// Display + touch + LVGL bring-up (was initDisplay() -> initLvgl()). The widget
// click handler (ui_on_click) is installed by main.ino's setup() AFTER this
// returns, since the app<->UI policy lives there.
void ui_init() {
  if (!USE_ROUND_DISPLAY) {
    return;
  }

  screen_rotation = 0;
  xiao_disp_init();          // tft.begin() + backlight + fillScreen
  tft.setTextWrap(false);
  initTouchButton();         // TOUCH_INT pinMode + Wire.begin()

  lv_init();
  lv_tick_set_cb(lv_millis_cb);

  g_disp = lv_display_create(240, 240);
  lv_display_set_buffers(g_disp, g_lvbuf, NULL, sizeof(g_lvbuf),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(g_disp, lv_flush_cb);

  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lv_touch_read_cb);

  // Build the initial screen onto the default active screen.
  lv_obj_t* scr = lv_screen_active();
  ui_build_home(scr, g_uiHomeAuto);
  g_uiCur = UI_HOME;
  g_uiReq = UI_HOME;

  g_lvglReady = true;
}
