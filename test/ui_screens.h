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

// Build each screen onto the given (already-created) screen object.
void ui_build_home(lv_obj_t* scr, bool autoMode);
void ui_build_countdown(lv_obj_t* scr, int secs, int totalSecs);
void ui_build_result(lv_obj_t* scr, const UiResult& r);
void ui_build_details(lv_obj_t* scr, const UiResult& r);

#ifdef __cplusplus
}
#endif
