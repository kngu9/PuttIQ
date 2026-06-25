#pragma once
// Application FSM + detection glue for PuttIQ. The swing state machine, gyro/
// gravity calibration, the legacy impact-gated detector that owns the result UI,
// the new PuttDetector diagnostics, stroke logging, and the app<->UI policy all
// live in app_fsm.cpp as private (static) state. main.ino sees only the thin
// app_* surface below; setup()/loop() are pure wiring.
#include "imu_driver.h"   // ImuSample, RawImuCounts
#include <cstdint>

// FSM/calibration/detector init: installs ui_click_fn = ui_on_click, seeds the
// home-face selector + status timers. Call from setup() AFTER ui_init().
void app_init();

// Power on + probe + configure the IMU (beginImu) and reset the FSM into its
// settling/calibration state on success, or the no-IMU state on failure.
// Returns true on success. Used by setup() and the loop() IMU-retry path.
bool app_start_imu_session();

// Print READY/NO-IMU once on a fresh USB-serial connection (edge-triggered).
void app_report_on_serial_connect(uint32_t nowMs);

// The whole !imuReady loop branch: heartbeat LED, periodic no-IMU status,
// periodic IMU re-probe, keep the (NO IMU) screen alive, then a short delay.
void app_no_imu_tick(uint32_t nowMs);

// IMU sample-rate gate (SAMPLE_PERIOD_US). Returns true and advances the
// schedule only when a new sample is due; false means skip this loop iteration.
bool app_sample_due(uint32_t nowUs);

// Called by loop() when readImu() fails: drop back into the no-IMU retry path.
void app_on_imu_read_failed();

// One per-loop tick for a freshly-read sample: raw-CSV logging, g_putt PUTTV2
// diagnostic, gravity/bias calibration, updateReady/updateSwing/updateHomeAnd
// Countdown, status print, ui_sync_from_state + ui_apply, and the activity LED.
void app_update(uint32_t nowMs, uint32_t nowUs, const ImuSample& sample, const RawImuCounts& counts);
