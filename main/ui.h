// On-device UI facade for PuttIQ: the LVGL display/touch pipeline plus the
// shared UI data the app reads/writes. The pure LVGL screen *builders* live in
// ui_screens.h/.cpp; the app<->UI *policy* (ui_on_click, ui_sync_from_state,
// g_inConfig) stays in main.ino. This header is the boundary between them.
#pragma once

#include "ui_screens.h"   // UiResult, UiEventId, ui_build_* builders, ui_click_fn

// Screen identity for the deferred screen-swap pipeline. Declared here (not in
// ui_screens.h) so the Arduino auto-prototype generator in main.ino sees the
// type before any prototype that uses it, and so both ui.cpp and main.ino share
// one definition. The pure builders in ui_screens.h don't need it.
enum UiScreen { UI_NONE, UI_HOME, UI_COUNTDOWN, UI_RESULT, UI_DETAILS, UI_CONFIG, UI_NOIMU, UI_SETTLING };

// Bring up the panel + touch + LVGL and load the initial home screen. Was
// initDisplay() + initLvgl() in main.ino.
void ui_init();

// Apply a pending screen request: build new, load it, delete the OLD active
// screen. SAFE only because it runs in loop(), never inside an event callback.
void ui_apply();

// Request a screen. Pass force=true to rebuild even when it's already current
// (used for the mode toggle and ZERO, which change content of the same kind).
void ui_request(UiScreen s, bool force = false);

// True once ui_init() has finished (LVGL is live). loop() gates lv_timer_handler
// and ui_apply() on this.
bool ui_ready();

// Shared UI data the app (main.ino) reads/writes. Kept as extern globals to
// mirror the pre-refactor layout (lowest-risk move).
extern UiScreen g_uiCur;          // currently-loaded screen (set by ui_apply)
extern int      g_countdownSecs;  // digit shown on UI_COUNTDOWN
extern int      g_countdownTotal; // countdown start digit
extern UiResult g_uiResult;       // populated by the app on an accepted putt
extern float    g_traceX[];       // backing storage for g_uiResult.traceX/Y
extern float    g_traceY[];

// Home-face selector read by ui_build_screen() for UI_HOME: true = the
// listening/auto face, false = the manual START home. The app keeps this current
// before any home rebuild (set in armForSwing/enterManualHome + at init), which
// preserves the old `state != SENSOR_HOME` behavior without a UI->app type dep.
extern bool     g_uiHomeAuto;
