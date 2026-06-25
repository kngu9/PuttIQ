// Platform-agnostic LVGL v9.5 screen builders for PuttIQ.
// These call ONLY LVGL APIs so they can be reused on-device. No host/PNG code here.
#pragma once

#ifndef LV_CONF_INCLUDE_SIMPLE
#define LV_CONF_INCLUDE_SIMPLE
#endif
#include "lvgl.h"

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Result payload for the trace/details screens.
struct UiResult {
    float faceDeg;        // face angle magnitude (deg)
    char  faceLR;         // 'L' or 'R'
    float pathDeg;        // path angle magnitude (deg)
    bool  pathOut;        // true = OUT (in-to-out), false = IN
    float tempo;          // backswing:downswing ratio (e.g. 2.1 => "2.1:1")
    uint32_t backMs;      // backswing duration
    uint32_t fwdMs;       // forward/downswing duration
    uint32_t durMs;       // total stroke duration
    int   impactOffMs;    // impact offset relative to some reference (signed ms)

    const float* traceX;  // clubhead trace X samples (arbitrary units)
    const float* traceY;  // clubhead trace Y samples
    int   traceCount;     // number of samples
    int   impactIndex;    // index into trace at impact
};

// Interactive-widget identifiers. The builders tag their clickable widgets with
// one of these via lv_obj_set_user_data(); a host (main.ino) attaches a single
// LV_EVENT_CLICKED handler that reads the id and sets app flags. Keeping the
// dispatch in the host means the builders stay pure/host-previewable.
enum UiEventId {
    UI_EVT_NONE = 0,
    UI_EVT_TOGGLE_AUTO,    // tapped the AUTO half of the mode toggle
    UI_EVT_TOGGLE_MAN,     // tapped the MAN half of the mode toggle
    UI_EVT_START,          // manual-home START button
    UI_EVT_ZERO,           // details-page ZERO button
    UI_EVT_RESULT_BODY,    // result screen background (advance to details)
    UI_EVT_DETAILS_BODY,   // details screen background (page back to result)
    UI_EVT_EXIT,           // EXIT affordance on result/details (back to home/idle)
    UI_EVT_CONFIG          // gear/CONFIG affordance on home (open config screen)
};

// Optional click dispatcher. When non-null, the builders attach an
// LV_EVENT_CLICKED callback to their interactive widgets that calls this with
// the widget's UiEventId. The host preview leaves it null (no interaction).
typedef void (*UiClickFn)(int id);
extern UiClickFn ui_click_fn;

// Build each screen onto the given (already-created) screen object.
void ui_build_home(lv_obj_t* scr, bool autoMode);
void ui_build_countdown(lv_obj_t* scr, int secs, int totalSecs);
void ui_build_result(lv_obj_t* scr, const UiResult& r);
void ui_build_details(lv_obj_t* scr, const UiResult& r);
void ui_build_config(lv_obj_t* scr);

#ifdef __cplusplus
}
#endif
