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

#include <cstdint>
#include <cstdio>
#include <cstring>

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

static void build_test_screen(void)
{
    lv_obj_t *scr = lv_screen_active();

    // Black background.
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    const lv_color_t amber = lv_color_hex(0xFFB000);

    // Arc: 0-270 deg sweep in amber.
    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 200, 200);
    lv_obj_center(arc);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_angles(arc, 0, 200);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);  // hide the draggable knob
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x303030), LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, amber, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 10, LV_PART_INDICATOR);

    // Filled amber circle (small obj with full radius).
    lv_obj_t *dot = lv_obj_create(scr);
    lv_obj_set_size(dot, 40, 40);
    lv_obj_align(dot, LV_ALIGN_CENTER, 0, 55);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(dot, amber, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(dot, 0, LV_PART_MAIN);

    // Centered "PuttIQ" label in white, Montserrat 28.
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "PuttIQ");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -20);
}

int main(void)
{
    lv_init();
    lv_tick_set_cb(my_tick);

    lv_display_t *disp = lv_display_create(W, H);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(disp, flush_cb);

    memset(fb, 0, sizeof(fb));

    build_test_screen();

    // Pump LVGL to force a render.
    for (int i = 0; i < 20; i++) {
        g_ms += 16;
        lv_timer_handler();
    }
    lv_refr_now(disp);

    const char *path = "build/preview_test.png";
    int ok = stbi_write_png(path, W, H, 3, fb, W * 3);
    if (!ok) {
        fprintf(stderr, "stbi_write_png failed for %s\n", path);
        return 1;
    }

    printf("PREVIEW_OK %s\n", path);
    return 0;
}
