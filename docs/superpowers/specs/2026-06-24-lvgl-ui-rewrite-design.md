# LVGL UI Rewrite + Detector Consolidation — Design

**Date:** 2026-06-24
**Status:** Approved (design), phased implementation
**Platform:** Seeed XIAO nRF52840 Sense, 240×240 round GC9A01, Arduino C++ + LVGL

## Goal

Replace the immediate-mode TFT_eSPI UI with a modern **LVGL** UI (anti-aliased
widgets, smooth arcs/gauges, real fonts, dirty-rectangle rendering that sidesteps the
full-frame ~1.5 FPS limit), consolidate detection into the pure host-tested module, and
restructure the monolithic `main.ino`.

## Hard constraint & verification strategy

The result is visual, and the developer agent cannot see the device screen or read its
serial. Mitigation:
- **Host PNG preview harness:** LVGL screen-construction code is platform-agnostic. A host
  build renders each screen into a 240×240 framebuffer and writes a PNG (via a tiny
  bundled PNG writer). Every screen is inspected as a PNG before flashing.
- Detection stays host-tested against recorded clips.
- Final on-device flash + visual confirmation is the developer's (human) step.

## Decisions (from brainstorming)

- **Restructure:** full — split `main.ino` into `app/`, `ui/`, `imu/` units; delete the dead
  legacy detection + all TFT_eSPI drawing.
- **Detector:** consolidate — extend the pure `PuttDetector` to emit a full `PuttResult`
  (trace points + face/path/tempo/durations/impact); retire the legacy `SwingStats` state
  machine entirely.
- **Visuals:** redesign with LVGL widgets — arc gauges for face/path/tempo, animated
  countdown ring, heartbeat pulse, smooth transitions.
- **Fonts:** Montserrat (LVGL built-in) for text; DSEG7 (converted) for big numeric readouts.
- **LVGL backend:** Arduino_GFX (Seeed-supported nRF52 path), partial draw buffer.
- **Sequencing:** incremental, validated per phase.

## Phases

### Phase B — Detector consolidation (do first; fully host-verifiable)
Extend `PuttDetector` so `decideWindow` produces a `PuttResult`:
```
struct PuttResult {
  bool valid; PuttDecision decision; const char* reason;
  float faceDeg, pathDeg;          // from OrientationTracker.decompose
  float tempo; uint32_t backswingMs, forwardMs, durationMs;
  float peakForwardDps; bool impactPresent; float impactJerk;
  uint16_t traceCount; HeadPoint trace[TRACE_CAP];   // from OrientationTracker.headPoint over the window
  uint16_t impactIndex;
};
```
- Trace: sample `OrientationTracker.headPoint(swingAxis, L)` across the window (downsampled
  to TRACE_CAP≈96 points).
- Tempo/backswing/forward: detect the back→forward reversal within the window (sign change
  of the on-axis projection) to split timing.
- Host tests: assert a synthetic pendulum yields a sane trace (forward sweep), face≈0 for a
  square stroke, plausible tempo; re-run the recorded-clip replay.
- Retire legacy `SwingStats`/`finishSwing`/`processBufferedSample` once the UI uses `PuttResult`.

### Phase A — LVGL pipeline bring-up
- Add LVGL; `lv_conf.h` enabling Montserrat sizes + DSEG7; Arduino_GFX flush callback;
  `lv_tick` from millis; touch input device from the CHSC6X driver.
- Drop the TFT_eSprite and TFT_eSPI drawing.
- One static "home" screen to confirm render + touch on-device.

### Phase C — LVGL screens (driven by `PuttResult`)
- **Home:** AUTO|MANUAL toggle (lv_switch/segmented), heartbeat pulse (lv_anim on an arc/dot),
  MANUAL START button.
- **Countdown:** animated 5→0 ring (lv_arc animated) + big DSEG7 digit.
- **Result — trace hero:** the clubhead arc (lv_canvas or line points) with the amber impact
  ball; face/path/tempo as small **arc gauges** around the rim; DSEG7 hero value.
- **Details:** face/path/tempo/B-F/dur/impact rows; `[ZERO]` calibration button.
- Verified via PNG previews.

### Phase D — Restructure & cleanup
- `main.ino` → thin wiring. `imu/` (driver+read), `app/` (mode FSM, countdown, calibration),
  `ui/` (LVGL screens + update fns). Delete legacy detection + TFT_eSPI code.

## File structure (target)
```
main/
  main.ino            # setup/loop wiring only
  imu_types.h, vec3.h, putt_preprocess.*, putt_features.*, putt_orientation.*, putt_detector.*  (pure, host-tested)
  app_fsm.h/.cpp      # modes, countdown, calibration, result routing
  ui.h/.cpp           # LVGL screens + update functions (platform-agnostic build)
  ui_fonts/           # dseg7 LVGL font .c ; lv_conf.h
  imu_driver.h/.cpp   # LSM6DS3 I2C read
test/
  ... existing host tests + test_result.cpp + ui_preview.cpp (renders screens to PNG)
```

## Out of scope
- VLW custom text fonts beyond Montserrat/DSEG7.
- Touch gesture changes beyond porting current taps/swipes.
