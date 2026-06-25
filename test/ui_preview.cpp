// HOST-ONLY LVGL -> PNG preview harness.
// Renders an LVGL v9.5 screen (240x240, RGB565) to build/preview_test.png so UI
// screens can be previewed on desktop without the embedded device.
//
// NOT part of the device firmware build and NOT part of `make test`.

#ifndef LV_CONF_INCLUDE_SIMPLE
#define LV_CONF_INCLUDE_SIMPLE
#endif
#include "lvgl.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "ui_screens.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

static const int W = 240;
static const int H = 240;

// Millisecond tick source for LVGL (no OS on host).
static uint32_t g_ms = 0;
static uint32_t my_tick(void) { return g_ms; }

// RGB888 framebuffer handed to stb for PNG encoding.
static uint8_t fb[W * H * 3];

// LVGL renders into this full-screen buffer in LV_COLOR_DEPTH 16 (RGB565).
static lv_color_t draw_buf[W * H];

// Flush callback: px_map is RGB565 little-endian (uint16 per pixel in v9.5).
static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    for (int32_t y = area->y1; y <= area->y2; y++) {
        for (int32_t x = area->x1; x <= area->x2; x++) {
            uint16_t c = (uint16_t)px_map[0] | ((uint16_t)px_map[1] << 8);
            px_map += 2;

            uint8_t r5 = (c >> 11) & 0x1F;
            uint8_t g6 = (c >> 5) & 0x3F;
            uint8_t b5 = c & 0x1F;

            // Expand 5/6-bit channels to 8-bit.
            uint8_t r8 = (uint8_t)((r5 << 3) | (r5 >> 2));
            uint8_t g8 = (uint8_t)((g6 << 2) | (g6 >> 4));
            uint8_t b8 = (uint8_t)((b5 << 3) | (b5 >> 2));

            if (x >= 0 && x < W && y >= 0 && y < H) {
                uint8_t *p = &fb[(y * W + x) * 3];
                p[0] = r8;
                p[1] = g8;
                p[2] = b8;
            }
        }
    }
    lv_display_flush_ready(disp);
}

static lv_display_t *g_disp = nullptr;

// Render whatever is on `s` and write it out as a PNG.
static int render_png(lv_obj_t *s, const char *path)
{
    lv_screen_load(s);
    memset(fb, 0, sizeof(fb));
    for (int i = 0; i < 20; i++) {
        g_ms += 16;
        lv_timer_handler();
    }
    lv_refr_now(g_disp);

    int ok = stbi_write_png(path, W, H, 3, fb, W * 3);
    if (!ok) {
        fprintf(stderr, "stbi_write_png failed for %s\n", path);
        return 1;
    }
    printf("PREVIEW_OK %s\n", path);
    return 0;
}

// Build the sample putting-arc trace: a mostly-horizontal sweep with slight
// upward curvature, impact near index 30.
static const int TRACE_N = 50;
static float traceX[TRACE_N];
static float traceY[TRACE_N];

static void make_sample_trace(void)
{
    // Match real PuttResult head-trace convention: traceY = forward sweep along
    // the target line (large range), traceX = small lateral arc (the curvature).
    // Realistic magnitudes from recorded clips: forward span ~0.34, lateral ~0.07.
    for (int i = 0; i < TRACE_N; i++) {
        float t = (float)i / (float)(TRACE_N - 1);   // 0..1
        float fwd = (t - 0.5f) * 0.34f;               // forward sweep -> traceY (span 0.34)
        float lat = 0.07f * (1.0f - 4.0f * (t - 0.5f) * (t - 0.5f)); // gentle arc, peak ~0.07
        traceX[i] = lat;
        traceY[i] = fwd;
    }
}

int main(void)
{
    lv_init();
    lv_tick_set_cb(my_tick);

    g_disp = lv_display_create(W, H);
    lv_display_set_buffers(g_disp, draw_buf, NULL, sizeof(draw_buf),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(g_disp, flush_cb);

    make_sample_trace();

    UiResult r;
    memset(&r, 0, sizeof(r));
    r.faceDeg = 1.2f; r.faceLR = 'R';
    r.pathDeg = 0.5f; r.pathOut = true;
    r.tempo = 2.1f;
    r.backMs = 480; r.fwdMs = 230; r.durMs = 710;
    r.impactOffMs = 180;
    r.traceX = traceX; r.traceY = traceY;
    r.traceCount = TRACE_N; r.impactIndex = 30;

    int rc = 0;

    lv_obj_t *s;
    s = lv_obj_create(NULL); ui_build_home(s, true);
    rc |= render_png(s, "build/screen_home_auto.png");

    s = lv_obj_create(NULL); ui_build_home(s, false);
    rc |= render_png(s, "build/screen_home_manual.png");

    s = lv_obj_create(NULL); ui_build_countdown(s, 3, 5);
    rc |= render_png(s, "build/screen_countdown.png");

    s = lv_obj_create(NULL); ui_build_result(s, r);
    rc |= render_png(s, "build/screen_result.png");

    s = lv_obj_create(NULL); ui_build_details(s, r);
    rc |= render_png(s, "build/screen_details.png");

    s = lv_obj_create(NULL); ui_build_config(s);
    rc |= render_png(s, "build/screen_config.png");

    return rc;
}
