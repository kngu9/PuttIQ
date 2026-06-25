// PuttIQ LVGL v9.5 screen builders. LVGL-only; reusable on-device.
#include "ui_screens.h"

#include <cstdio>
#include <cstring>
#include <cmath>

// DSEG7 7-segment fonts (generated via lv_font_conv).
// The generated .c files are compiled as C++ here (g++), so use C++ linkage.
extern const lv_font_t dseg7_44;
extern const lv_font_t dseg7_28;

// ---- Palette --------------------------------------------------------------
static const uint32_t COL_BLACK = 0x000000;
static const uint32_t COL_AMBER = 0xFFB000;
static const uint32_t COL_WHITE = 0xFFFFFF;
static const uint32_t COL_GREY  = 0x888888;
static const uint32_t COL_TRACK = 0x333333;

// ---- Type scale -----------------------------------------------------------
// One size per role; reuse everywhere so the same role looks identical.
#define FONT_CAPTION (&lv_font_montserrat_14)  // small grey sub-labels / hints
#define FONT_BODY    (&lv_font_montserrat_16)  // toggle, list rows, "listening", suffix
#define FONT_VALUE   (&dseg7_28)               // face / tempo numeric readouts
#define FONT_HERO    (&dseg7_44)               // countdown digit
#define FONT_BTN     (&lv_font_montserrat_28)  // START button label

// Screen geometry.
static const int CX = 120;
static const int CY = 120;

// ---- Interaction dispatch -------------------------------------------------
// Host (main.ino) installs this; preview leaves it null.
UiClickFn ui_click_fn = nullptr;

// Trampoline: pull the UiEventId out of user_data and forward to ui_click_fn.
static void ui_click_trampoline(lv_event_t* e)
{
    if (!ui_click_fn) return;
    lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
    int id = (int)(intptr_t)lv_obj_get_user_data(obj);
    ui_click_fn(id);
}

// Make `obj` a click target tagged with `id`.
static void make_clickable(lv_obj_t* obj, int id)
{
    lv_obj_set_user_data(obj, (void*)(intptr_t)id);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj, ui_click_trampoline, LV_EVENT_CLICKED, nullptr);
}

// Add a clearly-distinct pressed state to a button-like widget.
// `amberFill`: true for solid-amber buttons (dim slightly on press); false for
// outline buttons (fill faintly amber on press). Also scales down ~5%.
static void add_pressed_feedback(lv_obj_t* obj, bool amberFill)
{
    if (amberFill) {
        lv_obj_set_style_bg_opa(obj, LV_OPA_70, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        lv_obj_set_style_bg_color(obj, lv_color_hex(COL_AMBER),
                                  LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_opa(obj, LV_OPA_30, LV_PART_MAIN | LV_STATE_PRESSED);
    }
    lv_obj_set_style_transform_scale(obj, 243, LV_PART_MAIN | LV_STATE_PRESSED); // ~0.95 (256=1.0)
}

// Two-dot page indicator near the bottom of result/details (within the circle).
// `active` selects which dot is amber (0 = result, 1 = details).
static void build_page_dots(lv_obj_t* scr, int active, int y)
{
    const int gap = 14;            // center-to-center spacing
    const int d = 5;               // dot diameter
    for (int i = 0; i < 2; i++) {
        int x = CX + (i == 0 ? -gap / 2 : gap / 2);
        lv_obj_t* dot = lv_obj_create(scr);
        lv_obj_set_size(dot, d, d);
        lv_obj_align(dot, LV_ALIGN_CENTER, x - CX, y - CY);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(dot, lv_color_hex(i == active ? COL_AMBER : COL_GREY),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(dot, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(dot, 0, LV_PART_MAIN);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    }
}

// A small rounded "EXIT" pill at the bottom of result/details. Grey outline +
// grey label; expanded tap area so the effective target is >=44pt.
static void build_exit_pill(lv_obj_t* scr, int y)
{
    lv_obj_t* pill = lv_obj_create(scr);
    lv_obj_set_size(pill, 60, 26);
    lv_obj_align(pill, LV_ALIGN_CENTER, 0, y - CY);
    lv_obj_set_style_radius(pill, 13, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pill, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_color(pill, lv_color_hex(COL_GREY), LV_PART_MAIN);
    lv_obj_set_style_border_width(pill, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pill, 0, LV_PART_MAIN);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(pill, 12);   // 26 + 2*12 = 50pt effective tap height
    make_clickable(pill, UI_EVT_EXIT);
    add_pressed_feedback(pill, false);

    lv_obj_t* t = lv_label_create(pill);
    lv_label_set_text(t, "EXIT");
    lv_obj_set_style_text_font(t, FONT_CAPTION, LV_PART_MAIN);
    lv_obj_set_style_text_color(t, lv_color_hex(COL_GREY), LV_PART_MAIN);
    lv_obj_center(t);
}

// ---- Small helpers --------------------------------------------------------

static void bg_black(lv_obj_t* scr)
{
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BLACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 0, LV_PART_MAIN);
    // Kill any default scrollbars/scrolling so nothing clips oddly on a round panel.
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
}

// A label positioned by its center at (x,y) relative to screen origin.
static lv_obj_t* label_at(lv_obj_t* parent, const char* txt, const lv_font_t* font,
                          uint32_t color, int x, int y)
{
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_align(l, LV_ALIGN_CENTER, x - CX, y - CY);
    return l;
}

// A filled circle of radius r centered at (x,y).
static lv_obj_t* filled_circle(lv_obj_t* parent, int r, uint32_t color, int x, int y)
{
    lv_obj_t* c = lv_obj_create(parent);
    lv_obj_set_size(c, r * 2, r * 2);
    lv_obj_align(c, LV_ALIGN_CENTER, x - CX, y - CY);
    lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(c, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(c, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(c, 0, LV_PART_MAIN);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

// ---- Home -----------------------------------------------------------------

// AUTO|MAN segmented toggle at top center.
static void build_mode_toggle(lv_obj_t* scr, bool autoMode)
{
    const int W = 120, Hh = 40;
    lv_obj_t* box = lv_obj_create(scr);
    lv_obj_set_size(box, W, Hh);
    lv_obj_align(box, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_radius(box, 20, LV_PART_MAIN);
    lv_obj_set_style_bg_color(box, lv_color_hex(COL_BLACK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(box, lv_color_hex(COL_GREY), LV_PART_MAIN);
    lv_obj_set_style_border_width(box, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(box, 0, LV_PART_MAIN);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    const int segW = W / 2;

    // Active segment highlight (amber rounded fill).
    lv_obj_t* hi = lv_obj_create(box);
    lv_obj_set_size(hi, segW, Hh);
    lv_obj_align(hi, autoMode ? LV_ALIGN_LEFT_MID : LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_radius(hi, 20, LV_PART_MAIN);
    lv_obj_set_style_bg_color(hi, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(hi, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(hi, 0, LV_PART_MAIN);
    lv_obj_clear_flag(hi, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* a = lv_label_create(box);
    lv_label_set_text(a, "AUTO");
    lv_obj_set_style_text_font(a, FONT_BODY, LV_PART_MAIN);
    lv_obj_set_style_text_color(a, lv_color_hex(autoMode ? COL_BLACK : COL_GREY), LV_PART_MAIN);
    lv_obj_align(a, LV_ALIGN_LEFT_MID, 12, 0);

    lv_obj_t* m = lv_label_create(box);
    lv_label_set_text(m, "MAN");
    lv_obj_set_style_text_font(m, FONT_BODY, LV_PART_MAIN);
    lv_obj_set_style_text_color(m, lv_color_hex(autoMode ? COL_GREY : COL_BLACK), LV_PART_MAIN);
    lv_obj_align(m, LV_ALIGN_RIGHT_MID, -14, 0);

    // Two transparent click halves on top so each side is its own target.
    lv_obj_t* ta = lv_obj_create(box);
    lv_obj_set_size(ta, segW, Hh);
    lv_obj_align(ta, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_opa(ta, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ta, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ta, 0, LV_PART_MAIN);
    lv_obj_clear_flag(ta, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(ta, 10);   // 40 + 2*10 -> >=44pt effective tap
    make_clickable(ta, UI_EVT_TOGGLE_AUTO);
    add_pressed_feedback(ta, false);

    lv_obj_t* tm = lv_obj_create(box);
    lv_obj_set_size(tm, segW, Hh);
    lv_obj_align(tm, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_bg_opa(tm, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(tm, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tm, 0, LV_PART_MAIN);
    lv_obj_clear_flag(tm, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(tm, 10);   // 40 + 2*10 -> >=44pt effective tap
    make_clickable(tm, UI_EVT_TOGGLE_MAN);
    add_pressed_feedback(tm, false);
}

void ui_build_home(lv_obj_t* scr, bool autoMode)
{
    bg_black(scr);
    build_mode_toggle(scr, autoMode);

    if (autoMode) {
        // Amber filled circle r=12 centered at (120,110).
        filled_circle(scr, 12, COL_AMBER, CX, 110);
        label_at(scr, "listening", FONT_BODY, COL_GREY, CX, 150);
    } else {
        // Amber rounded START button.
        lv_obj_t* btn = lv_obj_create(scr);
        lv_obj_set_size(btn, 120, 54);
        lv_obj_align(btn, LV_ALIGN_CENTER, 0, 95 - CY);
        lv_obj_set_style_radius(btn, 16, LV_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COL_AMBER), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(btn, 0, LV_PART_MAIN);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_ext_click_area(btn, 8);
        make_clickable(btn, UI_EVT_START);
        add_pressed_feedback(btn, true);

        lv_obj_t* s = lv_label_create(btn);
        lv_label_set_text(s, "START");
        lv_obj_set_style_text_font(s, FONT_BTN, LV_PART_MAIN);
        lv_obj_set_style_text_color(s, lv_color_hex(COL_BLACK), LV_PART_MAIN);
        lv_obj_center(s);

        label_at(scr, "tap to begin", FONT_CAPTION, COL_GREY, CX, 160);
    }
}

// ---- Countdown ------------------------------------------------------------

void ui_build_countdown(lv_obj_t* scr, int secs, int totalSecs)
{
    bg_black(scr);

    if (totalSecs < 1) totalSecs = 1;
    if (secs < 0) secs = 0;
    if (secs > totalSecs) secs = totalSecs;

    // Ring: outer radius ~96 -> diameter 192, width ~10.
    lv_obj_t* arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 192, 192);
    lv_obj_center(arc);
    lv_arc_set_rotation(arc, 270);          // 0 deg of the arc points up (12 o'clock)
    lv_arc_set_bg_angles(arc, 0, 360);      // full-circle track
    int sweep = (int)((float)secs / (float)totalSecs * 360.0f + 0.5f);
    if (sweep < 1) sweep = 1;
    if (sweep > 360) sweep = 360;
    lv_arc_set_angles(arc, 0, sweep);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc, lv_color_hex(COL_TRACK), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(COL_AMBER), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_INDICATOR);

    label_at(scr, "GET READY", FONT_CAPTION, COL_GREY, CX, 58);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", secs);
    label_at(scr, buf, FONT_HERO, COL_AMBER, CX, CY + 6);
}

// ---- Result / trace -------------------------------------------------------

// Static storage for line points so they outlive the builder call.
static lv_point_precise_t s_approach[64];
static lv_point_precise_t s_follow[64];

// Scale the trace into a box of `boxPx` centered at (CX,CY) and split into
// approach (grey) + follow-through (white). Returns impact pixel coords.
static void build_trace(lv_obj_t* scr, const UiResult& r, int boxPx, int& impX, int& impY)
{
    impX = CX; impY = CY;
    if (!r.traceX || !r.traceY || r.traceCount < 2) return;

    int n = r.traceCount;
    if (n > 64) n = 64;

    // FIXED scale (px per trace unit), so stroke size is consistent and
    // comparable putt-to-putt instead of auto-fitting every stroke to the frame
    // (which amplified gentle strokes / integration noise into random squiggles).
    // Calibrated from recorded clips: a clean putt forward span ~0.34 -> ~110px.
    const float SCALE = 330.0f;
    const int   SAFE_R = 78;   // clamp drawn points to this radius from center

    // Light 3-tap smoothing to tame gyro-integration noise.
    static float sx[64], sy[64];
    for (int i = 0; i < n; i++) {
        int a = i > 0 ? i - 1 : i;
        int b = i < n - 1 ? i + 1 : i;
        sx[i] = (r.traceX[a] + r.traceX[i] + r.traceX[b]) / 3.0f;
        sy[i] = (r.traceY[a] + r.traceY[i] + r.traceY[b]) / 3.0f;
    }

    int imp = r.impactIndex;
    if (imp < 0) imp = 0;
    if (imp > n - 1) imp = n - 1;

    // Center on the impact point: the ball sits at screen center, path flows
    // through it. The stroke runs HORIZONTALLY: forward (sy) -> screen X
    // (approach left, follow-through right); lateral (sx) -> screen Y.
    const float ox = sx[imp], oy = sy[imp];
    const float fwdSign = (sy[n - 1] - sy[0]) < 0.0f ? -1.0f : 1.0f;

    auto toPx = [&](int i, lv_point_precise_t& p) {
        int px = CX + (int)((sy[i] - oy) * SCALE * fwdSign + 0.5f);
        int py = CY + (int)((sx[i] - ox) * SCALE + 0.5f);
        int dx = px - CX, dy = py - CY;
        int d2 = dx * dx + dy * dy;
        if (d2 > SAFE_R * SAFE_R) {          // clamp to the safe circle
            float k = SAFE_R / sqrtf((float)d2);
            px = CX + (int)(dx * k);
            py = CY + (int)(dy * k);
        }
        p.x = px;
        p.y = py;
    };

    int na = imp + 1;          // approach: 0..imp inclusive
    int nf = n - imp;          // follow:   imp..n-1 inclusive
    if (na > 64) na = 64;
    if (nf > 64) nf = 64;

    for (int i = 0; i < na; i++) toPx(i, s_approach[i]);
    for (int i = 0; i < nf; i++) toPx(imp + i, s_follow[i]);

    lv_obj_t* la = lv_line_create(scr);
    lv_line_set_points(la, s_approach, na);
    lv_obj_set_style_line_color(la, lv_color_hex(COL_GREY), LV_PART_MAIN);
    lv_obj_set_style_line_width(la, 3, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(la, true, LV_PART_MAIN);
    lv_obj_set_pos(la, 0, 0);

    lv_obj_t* lf = lv_line_create(scr);
    lv_line_set_points(lf, s_follow, nf);
    lv_obj_set_style_line_color(lf, lv_color_hex(COL_WHITE), LV_PART_MAIN);
    lv_obj_set_style_line_width(lf, 3, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(lf, true, LV_PART_MAIN);
    lv_obj_set_pos(lf, 0, 0);

    impX = (int)s_approach[imp < na ? imp : na - 1].x;
    impY = (int)s_approach[imp < na ? imp : na - 1].y;
}

void ui_build_result(lv_obj_t* scr, const UiResult& r)
{
    bg_black(scr);

    int impX, impY;
    build_trace(scr, r, 140, impX, impY);

    // Impact marker: amber ring (slightly larger outline) + filled amber dot.
    lv_obj_t* ring = lv_obj_create(scr);
    lv_obj_set_size(ring, 18, 18);
    lv_obj_align(ring, LV_ALIGN_CENTER, impX - CX, impY - CY);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_color(ring, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_set_style_border_width(ring, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ring, 0, LV_PART_MAIN);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
    filled_circle(scr, 6, COL_AMBER, impX, impY);

    // FACE block (top). Number + L/R suffix live in a centered flex row so the
    // pair reads as one balanced group (suffix vertically centered with digits).
    label_at(scr, "FACE", FONT_CAPTION, COL_GREY, CX, 34);
    char num[16];
    snprintf(num, sizeof(num), "%.1f", (double)r.faceDeg);
    lv_obj_t* frow = lv_obj_create(scr);
    lv_obj_remove_style_all(frow);
    lv_obj_set_size(frow, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(frow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(frow, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(frow, 4, 0);
    lv_obj_clear_flag(frow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(frow, LV_ALIGN_CENTER, 0, 56 - CY);
    lv_obj_t* fnum = lv_label_create(frow);
    lv_label_set_text(fnum, num);
    lv_obj_set_style_text_font(fnum, FONT_VALUE, 0);
    lv_obj_set_style_text_color(fnum, lv_color_hex(COL_AMBER), 0);
    char lr[2] = { r.faceLR, 0 };
    lv_obj_t* fsuf = lv_label_create(frow);
    lv_label_set_text(fsuf, lr);
    lv_obj_set_style_text_font(fsuf, FONT_BODY, 0);
    lv_obj_set_style_text_color(fsuf, lv_color_hex(COL_AMBER), 0);

    // TEMPO block (bottom). Value in DSEG7 to match the FACE readout style;
    // label stays Montserrat grey.
    char tempo[16];
    snprintf(tempo, sizeof(tempo), "%.1f:1", (double)r.tempo);
    label_at(scr, tempo, FONT_VALUE, COL_WHITE, CX, 150);
    label_at(scr, "TEMPO", FONT_CAPTION, COL_GREY, CX, 174);

    // Page indicator (result = dot 0) + EXIT affordance.
    build_page_dots(scr, 0, 188);
    build_exit_pill(scr, 216);

    // Tap anywhere on the result -> details.
    make_clickable(scr, UI_EVT_RESULT_BODY);
}

// ---- Details --------------------------------------------------------------

// One row: grey label at left edge, white value at right edge (within x52..188).
static void detail_row(lv_obj_t* scr, const char* label, const char* value, int y)
{
    const int xL = 52, xR = 188;
    lv_obj_t* l = lv_label_create(scr);
    lv_label_set_text(l, label);
    lv_obj_set_style_text_font(l, FONT_CAPTION, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_GREY), LV_PART_MAIN);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, xL, y);

    lv_obj_t* v = lv_label_create(scr);
    lv_label_set_text(v, value);
    lv_obj_set_style_text_font(v, FONT_BODY, LV_PART_MAIN);
    lv_obj_set_style_text_color(v, lv_color_hex(COL_WHITE), LV_PART_MAIN);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, xR - 240, y);
}

void ui_build_details(lv_obj_t* scr, const UiResult& r)
{
    bg_black(scr);

    // Tap anywhere (outside ZERO) -> back to home/idle. Attached first so the
    // ZERO button (added later, on top) wins its own taps.
    make_clickable(scr, UI_EVT_DETAILS_BODY);

    label_at(scr, "DETAILS", FONT_CAPTION, COL_GREY, CX, 38);

    char face[16], path[16], tempo[16], bf[24], dur[16], imp[16];
    snprintf(face,  sizeof(face),  "%.1f%c", (double)r.faceDeg, r.faceLR);
    snprintf(path,  sizeof(path),  "%.1f %s", (double)r.pathDeg, r.pathOut ? "OUT" : "IN");
    snprintf(tempo, sizeof(tempo), "%.1f:1", (double)r.tempo);
    snprintf(bf,    sizeof(bf),    "%u/%u", (unsigned)r.backMs, (unsigned)r.fwdMs);
    snprintf(dur,   sizeof(dur),   "%ums", (unsigned)r.durMs);
    snprintf(imp,   sizeof(imp),   "%+dms", r.impactOffMs);

    int y = 50;
    const int step = 14;
    detail_row(scr, "FACE",   face,  y); y += step;
    detail_row(scr, "PATH",   path,  y); y += step;
    detail_row(scr, "TEMPO",  tempo, y); y += step;
    detail_row(scr, "B-F",    bf,    y); y += step;
    detail_row(scr, "DUR",    dur,   y); y += step;
    detail_row(scr, "IMPACT", imp,   y); y += step;

    // [ZERO] button (enlarged for a comfortable touch target).
    lv_obj_t* btn = lv_obj_create(scr);
    lv_obj_set_size(btn, 90, 36);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 148 - CY);
    lv_obj_set_style_radius(btn, 18, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_ext_click_area(btn, 8);
    make_clickable(btn, UI_EVT_ZERO);
    add_pressed_feedback(btn, false);

    lv_obj_t* z = lv_label_create(btn);
    lv_label_set_text(z, "ZERO");
    lv_obj_set_style_text_font(z, FONT_BODY, LV_PART_MAIN);
    lv_obj_set_style_text_color(z, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_center(z);

    // Page indicator (details = dot 1) + EXIT affordance.
    build_page_dots(scr, 1, 182);
    build_exit_pill(scr, 212);
}
