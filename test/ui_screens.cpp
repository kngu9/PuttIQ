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

// Screen geometry.
static const int CX = 120;
static const int CY = 120;

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
    const int W = 120, Hh = 30;
    lv_obj_t* box = lv_obj_create(scr);
    lv_obj_set_size(box, W, Hh);
    lv_obj_align(box, LV_ALIGN_TOP_MID, 0, 22);
    lv_obj_set_style_radius(box, 15, LV_PART_MAIN);
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
    lv_obj_set_style_radius(hi, 15, LV_PART_MAIN);
    lv_obj_set_style_bg_color(hi, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(hi, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(hi, 0, LV_PART_MAIN);
    lv_obj_clear_flag(hi, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* a = lv_label_create(box);
    lv_label_set_text(a, "AUTO");
    lv_obj_set_style_text_font(a, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(a, lv_color_hex(autoMode ? COL_BLACK : COL_GREY), LV_PART_MAIN);
    lv_obj_align(a, LV_ALIGN_LEFT_MID, 12, 0);

    lv_obj_t* m = lv_label_create(box);
    lv_label_set_text(m, "MAN");
    lv_obj_set_style_text_font(m, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(m, lv_color_hex(autoMode ? COL_GREY : COL_BLACK), LV_PART_MAIN);
    lv_obj_align(m, LV_ALIGN_RIGHT_MID, -14, 0);
}

void ui_build_home(lv_obj_t* scr, bool autoMode)
{
    bg_black(scr);
    build_mode_toggle(scr, autoMode);

    if (autoMode) {
        // Amber filled circle r=12 centered at (120,110).
        filled_circle(scr, 12, COL_AMBER, CX, 110);
        label_at(scr, "listening", &lv_font_montserrat_20, COL_GREY, CX, 150);
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

        lv_obj_t* s = lv_label_create(btn);
        lv_label_set_text(s, "START");
        lv_obj_set_style_text_font(s, &lv_font_montserrat_28, LV_PART_MAIN);
        lv_obj_set_style_text_color(s, lv_color_hex(COL_BLACK), LV_PART_MAIN);
        lv_obj_center(s);

        label_at(scr, "tap to begin", &lv_font_montserrat_16, COL_GREY, CX, 160);
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

    label_at(scr, "GET READY", &lv_font_montserrat_16, COL_GREY, CX, 58);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", secs);
    label_at(scr, buf, &dseg7_44, COL_AMBER, CX, CY + 6);
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

    // Bounds.
    float minX = r.traceX[0], maxX = r.traceX[0];
    float minY = r.traceY[0], maxY = r.traceY[0];
    for (int i = 1; i < n; i++) {
        if (r.traceX[i] < minX) minX = r.traceX[i];
        if (r.traceX[i] > maxX) maxX = r.traceX[i];
        if (r.traceY[i] < minY) minY = r.traceY[i];
        if (r.traceY[i] > maxY) maxY = r.traceY[i];
    }
    float spanX = maxX - minX; if (spanX < 1e-6f) spanX = 1e-6f;
    float spanY = maxY - minY; if (spanY < 1e-6f) spanY = 1e-6f;
    // Uniform scale to preserve aspect; fit the larger span into boxPx.
    float span = spanX > spanY ? spanX : spanY;
    float scale = (float)boxPx / span;

    float midX = (minX + maxX) * 0.5f;
    float midY = (minY + maxY) * 0.5f;

    auto toPx = [&](int i, lv_point_precise_t& p) {
        // Center the data, scale, flip Y (screen Y grows downward).
        int px = CX + (int)((r.traceX[i] - midX) * scale + 0.5f);
        int py = CY - (int)((r.traceY[i] - midY) * scale + 0.5f);
        p.x = px;
        p.y = py;
    };

    int imp = r.impactIndex;
    if (imp < 0) imp = 0;
    if (imp > n - 1) imp = n - 1;

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

    // FACE block (top).
    label_at(scr, "FACE", &lv_font_montserrat_14, COL_GREY, CX, 34);
    char num[16];
    snprintf(num, sizeof(num), "%.1f", (double)r.faceDeg);
    // DSEG7 number, with the L/R suffix in Montserrat just to its right.
    lv_obj_t* fnum = lv_label_create(scr);
    lv_label_set_text(fnum, num);
    lv_obj_set_style_text_font(fnum, &dseg7_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(fnum, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_align(fnum, LV_ALIGN_CENTER, -8, 54 - CY);

    char lr[2] = { r.faceLR, 0 };
    lv_obj_t* fsuf = lv_label_create(scr);
    lv_label_set_text(fsuf, lr);
    lv_obj_set_style_text_font(fsuf, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(fsuf, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_align_to(fsuf, fnum, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, -2);

    // TEMPO block (bottom).
    char tempo[16];
    snprintf(tempo, sizeof(tempo), "%.1f:1", (double)r.tempo);
    label_at(scr, tempo, &lv_font_montserrat_20, COL_WHITE, CX, 150);
    label_at(scr, "TEMPO", &lv_font_montserrat_14, COL_GREY, CX, 172);
}

// ---- Details --------------------------------------------------------------

// One row: grey label at left edge, white value at right edge (within x52..188).
static void detail_row(lv_obj_t* scr, const char* label, const char* value, int y)
{
    const int xL = 52, xR = 188;
    lv_obj_t* l = lv_label_create(scr);
    lv_label_set_text(l, label);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(l, lv_color_hex(COL_GREY), LV_PART_MAIN);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, xL, y);

    lv_obj_t* v = lv_label_create(scr);
    lv_label_set_text(v, value);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(v, lv_color_hex(COL_WHITE), LV_PART_MAIN);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, xR - 240, y);
}

void ui_build_details(lv_obj_t* scr, const UiResult& r)
{
    bg_black(scr);

    label_at(scr, "DETAILS", &lv_font_montserrat_14, COL_GREY, CX, 38);

    char face[16], path[16], tempo[16], bf[24], dur[16], imp[16];
    snprintf(face,  sizeof(face),  "%.1f%c", (double)r.faceDeg, r.faceLR);
    snprintf(path,  sizeof(path),  "%.1f %s", (double)r.pathDeg, r.pathOut ? "OUT" : "IN");
    snprintf(tempo, sizeof(tempo), "%.1f:1", (double)r.tempo);
    snprintf(bf,    sizeof(bf),    "%u/%u", (unsigned)r.backMs, (unsigned)r.fwdMs);
    snprintf(dur,   sizeof(dur),   "%ums", (unsigned)r.durMs);
    snprintf(imp,   sizeof(imp),   "%+dms", r.impactOffMs);

    int y = 56;
    const int step = 15;
    detail_row(scr, "FACE",   face,  y); y += step;
    detail_row(scr, "PATH",   path,  y); y += step;
    detail_row(scr, "TEMPO",  tempo, y); y += step;
    detail_row(scr, "B-F",    bf,    y); y += step;
    detail_row(scr, "DUR",    dur,   y); y += step;
    detail_row(scr, "IMPACT", imp,   y); y += step;

    // [ZERO] button.
    lv_obj_t* btn = lv_obj_create(scr);
    lv_obj_set_size(btn, 70, 26);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 168 - CY);
    lv_obj_set_style_radius(btn, 13, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(btn, 0, LV_PART_MAIN);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* z = lv_label_create(btn);
    lv_label_set_text(z, "ZERO");
    lv_obj_set_style_text_font(z, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(z, lv_color_hex(COL_AMBER), LV_PART_MAIN);
    lv_obj_center(z);
}
