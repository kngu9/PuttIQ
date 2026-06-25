// Application FSM + detection glue for PuttIQ, extracted from main.ino.
//
// This owns the swing state machine, gyro/gravity calibration, the legacy
// impact-gated detector that drives the result UI, the new PuttDetector
// (g_putt) serial diagnostics, raw-stroke/CSV logging, and the app<->UI policy
// (ui_on_click / ui_sync_from_state). main.ino keeps only setup()/loop()
// wiring + printDisplayConfig() (which needs TFT_* macros).
//
// Everything here is private (static) except the handful of app_* entry points
// declared in app_fsm.h. This file writes the ui.h externs (g_uiResult, traces,
// countdown, g_uiHomeAuto) directly.
#include <Arduino.h>
#include <math.h>
#include "app_fsm.h"
#include "imu_driver.h"      // ImuSample / RawImuCounts / readImu / imuReady
#include "ui.h"              // ui_init/apply/request + shared UI data + UiScreen
#include "ui_screens.h"      // ui_build_* builders, UI_EVT_*, ui_click_fn
#include "imu_types.h"       // Vec3, RawSample, DetectorConfig, TRACE_CAP
#include "putt_detector.h"   // PuttDetector / PuttEvent
#include "putt_orientation.h" // OrientationTracker / StrokeAngles / HeadPoint

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// New impact-triggered detector (runs alongside the legacy path for live eval).
static PuttDetector g_putt{DetectorConfig{}};
// Orientation tracker: integrates the buffered stroke from the address pose so
// face (twist about shaft) and path (swing-plane tilt) are real, not estimated.
static OrientationTracker g_orient;
static bool g_orientActive = false;
static uint32_t orientLastMs = 0;
// Set by finishSwing(): true when the just-finished stroke was accepted. The
// buffered detection path reads this to know whether to build a UI result.
static bool g_lastFinishAccepted = false;
// Address-calibration offsets: tap "ZERO" on the stats page after a stroke you
// know was square/straight to null the systematic face/path bias.
static float faceZeroDeg = 0.0f;
static float pathZeroDeg = 0.0f;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

static const uint32_t SERIAL_BAUD = 115200;
static const char FW_VERSION[] = "face_zero_start_v71";

static const bool ENABLE_DEBUG_STATUS = false;
static const bool USE_ROUND_DISPLAY = true;
static const bool ENABLE_RAW_STROKE_LOG = true;

static const float RAD_TO_DEG_F = 57.2957795f;
static const float DEG_TO_RAD_F = 0.0174532925f;

static const uint16_t SAMPLE_HZ = 200;
static const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_HZ;
// 3 s of rolling history. This only feeds raw-CSV stroke logging now (the
// buffered analyzer is dead and the real-time detector uses live accumulators),
// so 5 s was ~104 KB of static RAM starving the LVGL heap. 3 s still covers a
// stroke for logging while freeing ~41 KB for screen allocation.
static const uint16_t SAMPLE_BUFFER_SIZE = SAMPLE_HZ * 3;
static const uint16_t TRACE_MAX_POINTS = 240;
static const uint32_t TRACE_POINT_MIN_MS = 10;

static const uint32_t BOOT_READY_HOLD_MS = 900;
static const uint32_t POST_PUTT_COOLDOWN_MS = 350;
static const uint32_t RESULT_MIN_HOLD_MS = 250;
static const uint32_t SWING_STOP_HOLD_MS = 260;
static const uint32_t POST_FORWARD_END_HOLD_MS = 180;
static const uint32_t MAX_PRETRIGGER_MS = 700;
static const uint32_t SHORT_CANDIDATE_EXPAND_MS = 650;
static const uint32_t READY_MIN_HOLD_MS = 250;
static const uint32_t MIN_ADDRESS_STILL_MS = 150;
static const uint32_t FSM_ARM_STILL_MS = 700;
static const uint32_t AXIS_LEARN_MS = 220;

static const uint32_t MIN_PUTT_DURATION_MS = 80;
static const uint32_t MAX_PUTT_DURATION_MS = 3300;
static const uint16_t MIN_PUTT_SAMPLES = 6;
static const uint32_t MIN_BACKSWING_MS = 10;
static const uint32_t MIN_FORWARD_MS = 20;

static const float STILL_GYRO_DPS = 5.0f;
static const float PRETRIGGER_GYRO_DPS = 0.9f;
static const float PRETRIGGER_RESET_GYRO_DPS = 0.8f;
static const float START_GYRO_DPS = 2.3f;
static const float REARM_MAX_GYRO_DPS = 6.0f;
static const float READY_STILL_GYRO_DPS = 6.0f;
static const float READY_STILL_LINEAR_MPS2 = 6.0f;
static const float START_LINEAR_MPS2 = 0.22f;
static const float PRETRIGGER_LINEAR_MPS2 = 0.09f;
static const float STOP_LINEAR_MPS2 = 0.18f;
static const float STOP_GYRO_DPS = 2.5f;
static const float GYRO_REVERSAL_DPS = 0.9f;
static const float MIN_PUTT_GYRO_DPS = 3.0f;
static const float FSM_START_GYRO_DPS = 4.0f;
static const float MIN_STROKE_GYRO_DPS = 8.0f;
static const float STRONG_STROKE_GYRO_DPS = 25.0f;
static const float MIN_STROKE_LINEAR_MPS2 = 0.22f;
static const float STRONG_STROKE_LINEAR_MPS2 = 3.0f;
static const float MIN_BACK_GYRO_DPS = 1.0f;
static const float MIN_FORWARD_GYRO_DPS = 1.4f;
static const float MIN_TEMPO_RATIO = 0.10f;
static const float MAX_TEMPO_RATIO = 8.00f;
static const float MIN_BACK_IMPULSE_DEG = 0.03f;
static const float MIN_FORWARD_IMPULSE_DEG = 0.05f;
static const float MIN_IMPULSE_RATIO = 0.01f;
static const float MAX_IMPULSE_RATIO = 80.00f;
static const float MIN_AXIS_QUALITY = 0.20f;
static const uint8_t MIN_PUTT_SCORE = 6;
static const float MAX_START_LINEAR_MPS2 = 99.0f;
static const float MAX_PUTT_LINEAR_MPS2 = 99.0f;
static const float ARMED_MIN_GYRO_DPS = 3.0f;
static const float ARMED_MIN_LINEAR_MPS2 = 0.18f;
static const float IMPACT_ACCEL_DELTA_MPS2 = 3.4f;
static const float GYRO_FILTER_ALPHA = 0.45f;
static const float GYRO_BIAS_ALPHA = 0.015f;
static const float FORWARD_END_FRACTION = 0.35f;
static const float HEAD_PATH_RADIUS = 1.0f;
static const float HEAD_PATH_ARC_DEPTH = 0.55f;
static const float HEAD_PATH_LATERAL_GAIN = 0.35f;
static const float GRAVITY_FILTER_ALPHA = 0.04f;
static const uint8_t START_CONFIRM_SAMPLES = 2;
static const uint8_t MAX_DIRECTION_CHANGES = 8;

// LSM6DS3 register map + scale constants moved to imu_driver.cpp (driver-private).

enum SensorState {
  SENSOR_NO_IMU,
  SENSOR_SETTLING,
  SENSOR_HOME,        // manual-mode idle: toggle + START button (no detection)
  SENSOR_COUNTDOWN,   // manual-mode 5s countdown before arming
  SENSOR_READY,
  SENSOR_SWING,
  SENSOR_RESULT_HOLD
};

enum AppMode { MODE_AUTO, MODE_MANUAL };
static const uint32_t MANUAL_COUNTDOWN_MS = 5000;

// enum UiScreen moved to ui.h (shared by ui.cpp + main.ino).

// Vec3 now comes from imu_types.h (shared with the new detector modules).
// ImuSample / RawImuCounts now come from imu_driver.h.

struct BufferedSample {
  uint32_t ms;
  Vec3 gyroRad;
  Vec3 linearAccelMps2;
  Vec3 accelMps2;
  float gyroDps;
  float linearMps2;
  bool valid;
};

struct SwingStats {
  uint32_t startMs;
  uint32_t transitionMs;
  uint32_t forwardEndMs;
  uint32_t lastMotionMs;
  uint16_t samples;
  Vec3 gyroAxis;
  Vec3 lateralAxis;
  Vec3 axisAccum;
  bool transitionDetected;
  bool forwardEndDetected;
  bool impactDetected;
  float maxGyroDps;
  float maxBackGyroDps;
  float maxForwardGyroDps;
  float backImpulseDeg;
  float forwardImpulseDeg;
  float axisQualitySum;
  float linearSumMps2;
  float maxLinearMps2;
  float maxAccelDeltaMps2;
  uint16_t axisQualitySamples;
  uint16_t linearSamples;
  uint32_t impactMs;
  Vec3 previousLinearAccelMps2;
  bool previousLinearAccelReady;
  uint32_t preStillMs;
  int8_t lastDirection;
  uint8_t directionChanges;
  uint32_t traceLastMs;
  float traceAngleRad;
  float traceLateralRad;
  uint32_t faceLastMs;
  float faceAngleDeg;
  float faceAngleImpactDeg;
  bool faceAngleImpactCaptured;
  float pathAngleImpactDeg;
};

enum ResultPage {
  RESULT_PAGE_TRACE,
  RESULT_PAGE_DETAILS,
  RESULT_PAGE_COUNT
};

struct LastResult {
  uint32_t durationMs;
  uint32_t backswingMs;
  uint32_t forwardMs;
  float tempo;
  float maxGyroDps;
  float maxForwardGyroDps;
  float maxLinearMps2;
  float maxAccelDeltaMps2;
  int score;
  bool impactDetected;
  int32_t impactOffsetMs;
  float faceAngleImpactDeg;
  bool faceAngleImpactCaptured;
  float pathAngleImpactDeg;
};

struct TracePoint {
  float x;
  float y;
};

static SensorState state = SENSOR_NO_IMU;
static AppMode appMode = MODE_AUTO;
static uint32_t countdownStartMs = 0;
static int countdownLastSec = -1;
// imuAddress / imuReady / imuTempSensitivity moved to imu_driver
// (imuReady is declared extern in imu_driver.h; the others are driver-private).

static Vec3 gyroBias = {0.0f, 0.0f, 0.0f};
static Vec3 filteredGyro = {0.0f, 0.0f, 0.0f};
static Vec3 gravityAxis = {0.0f, 0.0f, 1.0f};
static bool gyroBiasReady = false;
static bool gyroFilterReady = false;
static bool gravityReady = false;

static SwingStats swing;
static BufferedSample sampleBuffer[SAMPLE_BUFFER_SIZE];
static BufferedSample orderedBuffer[SAMPLE_BUFFER_SIZE];
static uint16_t sampleBufferNext = 0;
static uint16_t sampleBufferCount = 0;
static TracePoint tracePoints[TRACE_MAX_POINTS];
static uint16_t traceCount = 0;
static uint16_t traceTransitionIndex = 0;
static uint16_t traceImpactIndex = 0;
static float traceMinX = 0.0f;
static float traceMaxX = 0.0f;
static float traceMinY = 0.0f;
static float traceMaxY = 0.0f;

static uint32_t stateSinceMs = 0;
static uint32_t lastSampleUs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastImuRetryMs = 0;
static uint32_t readyStillSinceMs = 0;
static bool requireStillnessForReady = true;
static bool serialWasConnected = false;
static uint32_t resultPageShownMs = 0;
static float readyPeakGyroDps = 0.0f;
static uint8_t readyStartCount = 0;
static bool armedCaptureEvaluation = false;
static bool fsmArmed = false;
static ResultPage resultPage = RESULT_PAGE_TRACE;
static LastResult lastResult = {0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, false, -1, 0.0f, false};

static float mag3(const Vec3 &v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static float dot3(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 sub3(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 scale3(const Vec3 &v, float scale) {
  return {v.x * scale, v.y * scale, v.z * scale};
}

static Vec3 add3(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 cross3(const Vec3 &a, const Vec3 &b) {
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
}

static Vec3 normalize3(const Vec3 &v) {
  float m = mag3(v);
  if (m < 0.0001f) {
    return {1.0f, 0.0f, 0.0f};
  }
  return scale3(v, 1.0f / m);
}

static Vec3 perpendicular3(const Vec3 &v) {
  Vec3 p = {-v.y, v.x, 0.0f};
  if (mag3(p) < 0.0001f) {
    p = {1.0f, 0.0f, 0.0f};
  }
  return normalize3(p);
}

static float gyroDps(const Vec3 &gyroRad) {
  return mag3(gyroRad) * RAD_TO_DEG_F;
}

static void updateGravityEstimate(const Vec3 &accelMps2) {
  float accelMag = mag3(accelMps2);
  if (accelMag < 7.0f || accelMag > 12.5f) {
    return;
  }

  Vec3 accelAxis = normalize3(accelMps2);
  if (!gravityReady) {
    gravityAxis = accelAxis;
    gravityReady = true;
    return;
  }

  gravityAxis = normalize3(add3(scale3(gravityAxis, 1.0f - GRAVITY_FILTER_ALPHA),
                                scale3(accelAxis, GRAVITY_FILTER_ALPHA)));
}

static Vec3 linearAccelMps2(const Vec3 &accelMps2) {
  if (!gravityReady) {
    return {0.0f, 0.0f, 0.0f};
  }

  return sub3(accelMps2, scale3(gravityAxis, dot3(accelMps2, gravityAxis)));
}

static uint32_t clampU32(uint32_t value, uint32_t low, uint32_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

// ---------------------------------------------------------------------------
// The LVGL display/touch pipeline, the deferred screen-swap helpers
// (ui_request/ui_build_screen/ui_apply), and the UI data globals
// (g_uiCur/g_countdownSecs/g_countdownTotal/g_uiResult/g_traceX/Y, plus the
// new g_uiHomeAuto) now live in ui.cpp (declared in ui.h). The legacy
// immediate-mode TFT draw/touch helpers were dead code and were removed with
// the move. Only the app<->UI policy (below) stays here.
// ---------------------------------------------------------------------------

// Config is a UI-only modal over the idle state (no backing FSM state). While
// open, suppress the state->screen auto-sync and result pop-ups so it isn't
// bounced back to home each loop.
static bool g_inConfig = false;
// Raw (un-zeroed) face/path of the last accepted putt, for the ZERO button.
static float    g_lastRawFaceDeg = 0.0f;
static float    g_lastRawPathDeg = 0.0f;
static bool     g_haveLastResult = false;

// Forward decls for app helpers used by the click handler (defined below).
static void armForSwing(uint32_t nowMs);
static void enterManualHome(uint32_t nowMs);
static void enterIdle(uint32_t nowMs);
static void resetBuffer();

// Click dispatch from ui_screens widgets. Flags ONLY — no screen ops here.
static void ui_on_click(int id) {
  uint32_t nowMs = millis();
  switch (id) {
    case UI_EVT_TOGGLE_AUTO:
      if (appMode != MODE_AUTO) {
        appMode = MODE_AUTO;
        armForSwing(nowMs);          // auto: arm & listen immediately
        ui_request(UI_HOME, true);
      }
      break;
    case UI_EVT_TOGGLE_MAN:
      if (appMode != MODE_MANUAL) {
        appMode = MODE_MANUAL;
        enterManualHome(nowMs);      // manual: drop to START home
        ui_request(UI_HOME, true);
      }
      break;
    case UI_EVT_START:
      if (state == SENSOR_HOME) {
        state = SENSOR_COUNTDOWN;
        countdownStartMs = nowMs;
        countdownLastSec = -1;
        g_countdownSecs = g_countdownTotal;
        ui_request(UI_COUNTDOWN, true);
      }
      break;
    case UI_EVT_ZERO:
      if (g_haveLastResult) {
        faceZeroDeg = g_lastRawFaceDeg;
        pathZeroDeg = g_lastRawPathDeg;
        // Recompute the displayed result against the new zero.
        g_uiResult.faceDeg = fabsf(g_lastRawFaceDeg - faceZeroDeg);
        g_uiResult.faceLR  = (g_lastRawFaceDeg - faceZeroDeg) >= 0.0f ? 'R' : 'L';
        g_uiResult.pathDeg = fabsf(g_lastRawPathDeg - pathZeroDeg);
        g_uiResult.pathOut = (g_lastRawPathDeg - pathZeroDeg) >= 0.0f;
      }
      // ZERO now lives on the Config screen; stay on Config after calibrating.
      ui_request(UI_CONFIG, true);
      break;
    case UI_EVT_CONFIG:
      g_inConfig = true;             // modal: hold Config until EXIT
      ui_request(UI_CONFIG, true);
      break;
    case UI_EVT_RESULT_BODY:
      if (state == SENSOR_RESULT_HOLD) {
        ui_request(UI_DETAILS);
      }
      break;
    case UI_EVT_DETAILS_BODY:
      if (state == SENSOR_RESULT_HOLD) {
        ui_request(UI_RESULT, true); // page back to result
      }
      break;
    case UI_EVT_EXIT:
      // Shared EXIT affordance. From result/details (RESULT_HOLD) it dismisses
      // the result; from Config it returns to home. Either way -> idle/home.
      if (state == SENSOR_RESULT_HOLD || g_uiCur == UI_CONFIG) {
        g_inConfig = false;         // explicit exit clears the Config modal
        enterIdle(nowMs);            // auto: re-arm; manual: back to home
      }
      break;
    default:
      break;
  }
}

// Map the app FSM -> the screen that should be showing, and request it. Called
// from loop() (which then runs ui_apply()). RESULT_HOLD is intentionally NOT
// mapped here: results are page-able (RESULT <-> DETAILS) under user control, so
// the result screen is requested explicitly when a putt is accepted, and screen
// changes within RESULT_HOLD are driven by tap events.
static void ui_sync_from_state(uint32_t nowMs) {
  if (g_inConfig) return;   // Config is user-controlled; don't auto-sync over it.
  switch (state) {
    case SENSOR_NO_IMU:
      ui_request(UI_NOIMU);
      break;
    case SENSOR_SETTLING:
      // Settling is a brief calibration; show the listening home (not a bare
      // "...") so the idle screen reads consistently.
      ui_request(UI_HOME);
      break;
    case SENSOR_HOME:
      ui_request(UI_HOME);   // manual idle: toggle + START
      break;
    case SENSOR_COUNTDOWN: {
      // Advance the countdown digit; rebuild only when the second changes.
      uint32_t elapsed = nowMs - countdownStartMs;
      int secs = (int)((MANUAL_COUNTDOWN_MS - elapsed) / 1000) + 1;
      if (secs > g_countdownTotal) secs = g_countdownTotal;
      if (secs < 1) secs = 1;
      if (secs != g_countdownSecs || g_uiCur != UI_COUNTDOWN) {
        g_countdownSecs = secs;
        ui_request(UI_COUNTDOWN, true);
      }
      break;
    }
    case SENSOR_READY:
      // Both auto-home (listening) and manual armed-and-waiting show the
      // listening home (auto-style). Toggle stays live only in auto (gated in
      // updateHomeAndCountdown / the click handler).
      ui_request(UI_HOME);
      break;
    case SENSOR_SWING:
      // Mid-stroke: keep whatever is showing (listening home). No change.
      break;
    case SENSOR_RESULT_HOLD:
      // Result/details are user-paged; do not override here.
      break;
  }
}

// Fallback result builder for the REAL-TIME finishSwing() path (no buffered
// window). The buffered path is the primary detector; this only fires if the
// live FSM accepts a stroke that the buffered path didn't. It reuses the legacy
// estimated impact angles (lastResult.faceAngleImpactDeg/pathAngleImpactDeg) and
// the live tracePoints[] so the result is not lost. Mirrors the zero/R-L/OUT-IN
// conventions used by the on-screen stats (lastResult.* - faceZeroDeg, etc.).
static void buildLegacyUiResultRealtime(uint32_t nowMs) {
  if (g_inConfig) return;   // don't pop a result over the Config screen
  // Face/path from the OrientationTracker integrated live over the stroke. If
  // impact was detected we use the pose captured AT impact; otherwise decompose
  // the final accumulated orientation (sign convention: open=R, closed=L).
  float fr, pr;
  if (swing.impactDetected && swing.faceAngleImpactCaptured) {
    fr = swing.faceAngleImpactDeg;   // already sign-converted (open=R)
    pr = swing.pathAngleImpactDeg;
  } else {
    StrokeAngles a = g_orient.decompose(swing.gyroAxis);
    fr = -a.faceDeg;
    pr = a.pathDeg;
  }
  g_orientActive = false;
  g_uiResult.faceDeg = fabsf(fr - faceZeroDeg);
  g_uiResult.faceLR  = (fr - faceZeroDeg) >= 0.0f ? 'R' : 'L';
  g_uiResult.pathDeg = fabsf(pr - pathZeroDeg);
  g_uiResult.pathOut = (pr - pathZeroDeg) >= 0.0f;

  // Downsample the live trace into g_traceX/g_traceY.
  int n = (int)traceCount;
  if (n > 0) {
    int stride = (n + TRACE_CAP - 1) / TRACE_CAP;
    if (stride < 1) stride = 1;
    int outCount = 0;
    int impactOut = 0;
    for (int i = 0; i < n && outCount < TRACE_CAP; i += stride) {
      g_traceX[outCount] = tracePoints[i].x;
      g_traceY[outCount] = tracePoints[i].y;
      if ((uint16_t)i <= traceImpactIndex) impactOut = outCount;
      outCount++;
    }
    g_uiResult.traceCount = outCount;
    g_uiResult.impactIndex = impactOut < outCount ? impactOut : (outCount > 0 ? outCount - 1 : 0);
  } else {
    g_uiResult.traceCount = 0;
    g_uiResult.impactIndex = 0;
  }
  g_uiResult.traceX = g_traceX;
  g_uiResult.traceY = g_traceY;

  g_uiResult.tempo  = lastResult.tempo;
  g_uiResult.backMs = lastResult.backswingMs;
  g_uiResult.fwdMs  = lastResult.forwardMs;
  g_uiResult.durMs  = lastResult.durationMs;
  g_uiResult.impactOffMs =
      swing.impactDetected ? (int)(swing.impactMs - swing.startMs) : 0;

  g_lastRawFaceDeg = fr;
  g_lastRawPathDeg = pr;
  g_haveLastResult = true;

  state = SENSOR_RESULT_HOLD;
  stateSinceMs = nowMs;
  requireStillnessForReady = false;
  resetBuffer();
  ui_request(UI_RESULT, true);
}

// initDisplay()/initLvgl() moved to ui.cpp as ui_init(). The widget click
// handler (ui_on_click) is installed from setup() after ui_init() returns, since
// the app<->UI policy stays in this file.

// The hardware half of IMU bring-up (power/probe/configure) now lives in
// imu_driver (beginImu). This wrapper keeps the application-state resets that
// used to live inside beginImu(): on success it arms the settling state, on
// failure it drops to the no-IMU state. Behavior is identical to the old
// monolithic beginImu(); only the hardware code was extracted.
bool app_start_imu_session() {
  if (!beginImu()) {
    state = SENSOR_NO_IMU;
    return false;
  }

  state = SENSOR_SETTLING;
  stateSinceMs = millis();
  requireStillnessForReady = true;
  gyroBiasReady = false;
  gyroFilterReady = false;
  lastSampleUs = micros();
  return true;
}

static void updateGyroBias(const Vec3 &rawGyro) {
  if (!gyroBiasReady) {
    gyroBias = rawGyro;
    gyroBiasReady = true;
    return;
  }

  gyroBias = add3(scale3(gyroBias, 1.0f - GYRO_BIAS_ALPHA), scale3(rawGyro, GYRO_BIAS_ALPHA));
}

static Vec3 filteredGyroSample(const Vec3 &rawGyro) {
  Vec3 corrected = sub3(rawGyro, gyroBiasReady ? gyroBias : Vec3{0.0f, 0.0f, 0.0f});

  if (!gyroFilterReady) {
    filteredGyro = corrected;
    gyroFilterReady = true;
  } else {
    filteredGyro = add3(scale3(filteredGyro, 1.0f - GYRO_FILTER_ALPHA), scale3(corrected, GYRO_FILTER_ALPHA));
  }

  return filteredGyro;
}

static void resetBuffer() {
  sampleBufferNext = 0;
  sampleBufferCount = 0;
  for (uint16_t i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
    sampleBuffer[i].valid = false;
  }
}

static void resetTrace() {
  traceCount = 0;
  traceTransitionIndex = 0;
  traceImpactIndex = 0;
  traceMinX = 0.0f;
  traceMaxX = 0.0f;
  traceMinY = 0.0f;
  traceMaxY = 0.0f;
}

static void pushTrace(float x, float y) {
  if (traceCount >= TRACE_MAX_POINTS) {
    return;
  }

  tracePoints[traceCount] = {x, y};
  if (traceCount == 0) {
    traceMinX = x;
    traceMaxX = x;
    traceMinY = y;
    traceMaxY = y;
  } else {
    if (x < traceMinX) {
      traceMinX = x;
    }
    if (x > traceMaxX) {
      traceMaxX = x;
    }
    if (y < traceMinY) {
      traceMinY = y;
    }
    if (y > traceMaxY) {
      traceMaxY = y;
    }
  }
  traceCount++;
}

static void pushBuffer(uint32_t nowMs, const Vec3 &gyroRad, const Vec3 &linearAccelMps2, const Vec3 &accelMps2, float dps, float linearMps2) {
  sampleBuffer[sampleBufferNext] = {nowMs, gyroRad, linearAccelMps2, accelMps2, dps, linearMps2, true};
  sampleBufferNext = (sampleBufferNext + 1) % SAMPLE_BUFFER_SIZE;
  if (sampleBufferCount < SAMPLE_BUFFER_SIZE) {
    sampleBufferCount++;
  }
}

static uint32_t bufferedSwingStartMs(uint32_t triggerMs) {
  uint32_t candidateMs = triggerMs;
  bool foundMotion = false;

  for (uint16_t offset = 0; offset < sampleBufferCount; offset++) {
    int index = (int)sampleBufferNext - 1 - offset;
    while (index < 0) {
      index += SAMPLE_BUFFER_SIZE;
    }

    const BufferedSample &s = sampleBuffer[index];
    if (!s.valid) {
      continue;
    }

    if (triggerMs - s.ms > MAX_PRETRIGGER_MS) {
      break;
    }

    if (s.gyroDps > PRETRIGGER_GYRO_DPS) {
      candidateMs = s.ms;
      foundMotion = true;
    } else if (foundMotion && s.gyroDps < PRETRIGGER_RESET_GYRO_DPS) {
      break;
    }
  }

  return foundMotion ? candidateMs : triggerMs;
}

static uint16_t copyOrderedBuffer() {
  for (uint16_t i = 0; i < sampleBufferCount; i++) {
    uint16_t index = (sampleBufferNext + SAMPLE_BUFFER_SIZE - sampleBufferCount + i) % SAMPLE_BUFFER_SIZE;
    orderedBuffer[i] = sampleBuffer[index];
  }
  return sampleBufferCount;
}

static void resetSwing(uint32_t startMs) {
  swing.startMs = startMs;
  swing.transitionMs = startMs;
  swing.forwardEndMs = startMs;
  swing.lastMotionMs = startMs;
  swing.samples = 0;
  swing.gyroAxis = {1.0f, 0.0f, 0.0f};
  swing.lateralAxis = {0.0f, 1.0f, 0.0f};
  swing.axisAccum = {0.0f, 0.0f, 0.0f};
  swing.transitionDetected = false;
  swing.forwardEndDetected = false;
  swing.impactDetected = false;
  swing.maxGyroDps = 0.0f;
  swing.maxBackGyroDps = 0.0f;
  swing.maxForwardGyroDps = 0.0f;
  swing.backImpulseDeg = 0.0f;
  swing.forwardImpulseDeg = 0.0f;
  swing.axisQualitySum = 0.0f;
  swing.linearSumMps2 = 0.0f;
  swing.maxLinearMps2 = 0.0f;
  swing.maxAccelDeltaMps2 = 0.0f;
  swing.axisQualitySamples = 0;
  swing.linearSamples = 0;
  swing.impactMs = startMs;
  swing.previousLinearAccelMps2 = {0.0f, 0.0f, 0.0f};
  swing.previousLinearAccelReady = false;
  swing.preStillMs = 0;
  swing.lastDirection = 0;
  swing.directionChanges = 0;
  swing.traceLastMs = startMs;
  swing.traceAngleRad = 0.0f;
  swing.traceLateralRad = 0.0f;
  swing.faceLastMs = startMs;
  swing.faceAngleDeg = 0.0f;
  swing.faceAngleImpactDeg = 0.0f;
  swing.faceAngleImpactCaptured = false;
  swing.pathAngleImpactDeg = 0.0f;
  resetTrace();
  pushTrace(0.0f, 0.0f);
}

static void zeroFaceAngleAt(uint32_t zeroMs) {
  swing.faceLastMs = zeroMs;
  swing.faceAngleDeg = 0.0f;
  swing.faceAngleImpactDeg = 0.0f;
  swing.faceAngleImpactCaptured = false;
  swing.pathAngleImpactDeg = 0.0f;
}

static void printReadyEvent(uint32_t nowMs) {
  Serial.print(F("READY_TO_PUTT,ms="));
  Serial.println(nowMs);
}

static void armForSwing(uint32_t nowMs) {
  resetSwing(nowMs);
  resetBuffer();
  state = SENSOR_READY;
  stateSinceMs = nowMs;
  readyStillSinceMs = 0;
  readyPeakGyroDps = 0.0f;
  readyStartCount = 0;
  fsmArmed = true;  // ready == armed: no separate stillness-arm step
  g_uiHomeAuto = true;  // armed & listening -> show the auto/listening home face

  printReadyEvent(nowMs);
}

// Manual-mode idle home (no detection until START -> countdown -> arm).
static void enterManualHome(uint32_t nowMs) {
  resetBuffer();
  state = SENSOR_HOME;
  stateSinceMs = nowMs;
  fsmArmed = false;
  g_uiHomeAuto = false;  // manual idle -> show the START home face
}

// In auto, arm immediately; in manual, drop to the manual home.
static void enterIdle(uint32_t nowMs) {
  if (appMode == MODE_MANUAL) {
    enterManualHome(nowMs);
  } else {
    armForSwing(nowMs);
  }
}

static uint32_t forwardMsFor(uint32_t nowMs) {
  if (!swing.transitionDetected || swing.transitionMs <= swing.startMs) {
    return nowMs - swing.startMs;
  }

  uint32_t endMs = nowMs;
  if (swing.forwardEndDetected) {
    endMs = swing.forwardEndMs;
  } else if (nowMs > swing.transitionMs + SWING_STOP_HOLD_MS) {
    endMs = nowMs - SWING_STOP_HOLD_MS;
  }

  return endMs > swing.transitionMs ? endMs - swing.transitionMs : 1;
}

static int puttScore(uint32_t durationMs, uint32_t backswingMs, uint32_t forwardMs);

static void printResult(const char *prefix, const char *reason, uint32_t nowMs) {
  uint32_t durationMs = nowMs - swing.startMs;
  uint32_t backswingMs = swing.transitionDetected ? swing.transitionMs - swing.startMs : 0;
  uint32_t forwardMs = forwardMsFor(nowMs);
  float tempo = forwardMs > 0 ? (float)backswingMs / (float)forwardMs : 0.0f;

  Serial.print(prefix);
  if (reason != nullptr) {
    Serial.print(F(",reason="));
    Serial.print(reason);
  }
  Serial.print(F(",duration_ms="));
  Serial.print(durationMs);
  Serial.print(F(",backswing_ms="));
  Serial.print(backswingMs);
  Serial.print(F(",forward_ms="));
  Serial.print(forwardMs);
  Serial.print(F(",forward_end="));
  Serial.print(swing.forwardEndDetected ? F("decel") : F("quiet_fallback"));
  Serial.print(F(",tempo_ratio="));
  Serial.print(tempo, 2);
  Serial.print(F(",tempo="));
  Serial.print(tempo, 2);
  Serial.print(F(":1,max_gyro_dps="));
  Serial.print(swing.maxGyroDps, 1);
  Serial.print(F(",max_forward_gyro_dps="));
  Serial.print(swing.maxForwardGyroDps, 1);
  Serial.print(F(",max_linear_mps2="));
  Serial.print(swing.maxLinearMps2, 2);
  Serial.print(F(",impact_ms="));
  Serial.print(swing.impactDetected ? swing.impactMs : 0);
  Serial.print(F(",impact_offset_ms="));
  Serial.print(swing.impactDetected ? (long)(swing.impactMs - swing.startMs) : -1);
  Serial.print(F(",face_angle_impact_deg="));
  Serial.print(swing.faceAngleImpactCaptured ? swing.faceAngleImpactDeg : swing.faceAngleDeg, 1);
  Serial.print(F(",face_angle_source="));
  Serial.print(swing.faceAngleImpactCaptured ? F("impact") : F("estimate_end"));
  Serial.print(F(",face_angle_zero=stroke_start"));
  Serial.print(F(",max_accel_delta_mps2="));
  Serial.print(swing.maxAccelDeltaMps2, 2);
  Serial.print(F(",axis_quality="));
  Serial.print(swing.axisQualitySamples > 0 ? swing.axisQualitySum / swing.axisQualitySamples : 0.0f, 2);
  Serial.print(F(",back_impulse_deg="));
  Serial.print(swing.backImpulseDeg, 1);
  Serial.print(F(",forward_impulse_deg="));
  Serial.print(swing.forwardImpulseDeg, 1);
  Serial.print(F(",dir_changes="));
  Serial.print(swing.directionChanges);
  Serial.print(F(",linear_avg_mps2="));
  Serial.print(swing.linearSamples > 0 ? swing.linearSumMps2 / swing.linearSamples : 0.0f, 2);
  Serial.print(F(",pre_still_ms="));
  Serial.print(swing.preStillMs);
  Serial.print(F(",score="));
  Serial.print(puttScore(durationMs, backswingMs, forwardMs));
  Serial.print(F(",samples="));
  Serial.println(swing.samples);
}

static void logRawStrokeWindow(uint16_t startIndex, uint16_t endIndex) {
  if (!ENABLE_RAW_STROKE_LOG) {
    return;
  }

  Serial.print(F("RAW_STROKE_BEGIN,samples="));
  Serial.println(endIndex >= startIndex ? endIndex - startIndex + 1 : 0);
  Serial.println(F("RAW,dt_ms,ax,ay,az,lx,ly,lz,gx_dps,gy_dps,gz_dps"));
  uint32_t baseMs = orderedBuffer[startIndex].ms;
  for (uint16_t i = startIndex; i <= endIndex; i++) {
    const BufferedSample &s = orderedBuffer[i];
    Serial.print(F("RAW,"));
    Serial.print(s.ms - baseMs);
    Serial.print(',');
    Serial.print(s.accelMps2.x, 3);
    Serial.print(',');
    Serial.print(s.accelMps2.y, 3);
    Serial.print(',');
    Serial.print(s.accelMps2.z, 3);
    Serial.print(',');
    Serial.print(s.linearAccelMps2.x, 3);
    Serial.print(',');
    Serial.print(s.linearAccelMps2.y, 3);
    Serial.print(',');
    Serial.print(s.linearAccelMps2.z, 3);
    Serial.print(',');
    Serial.print(s.gyroRad.x * RAD_TO_DEG_F, 2);
    Serial.print(',');
    Serial.print(s.gyroRad.y * RAD_TO_DEG_F, 2);
    Serial.print(',');
    Serial.println(s.gyroRad.z * RAD_TO_DEG_F, 2);
  }
  Serial.println(F("RAW_STROKE_END"));
}

static void logRawStrokeRecent(uint32_t startMs, uint32_t endMs) {
  if (!ENABLE_RAW_STROKE_LOG) {
    return;
  }

  uint16_t count = copyOrderedBuffer();
  int16_t startIndex = -1;
  int16_t endIndex = -1;
  uint32_t logStartMs = startMs > 250 ? startMs - 250 : 0;
  uint32_t logEndMs = endMs + 150;

  for (uint16_t i = 0; i < count; i++) {
    if (orderedBuffer[i].ms >= logStartMs && startIndex < 0) {
      startIndex = i;
    }
    if (orderedBuffer[i].ms <= logEndMs) {
      endIndex = i;
    }
  }

  if (startIndex >= 0 && endIndex >= startIndex) {
    logRawStrokeWindow((uint16_t)startIndex, (uint16_t)endIndex);
  }
}

// Continuous per-sample raw CSV for offline corpus capture:
//   t_us,gx,gy,gz,ax,ay,az   (raw register counts)
static void logRawCsvSample(uint32_t tUs, const RawImuCounts &c) {
  if (!ENABLE_RAW_STROKE_LOG) {
    return;
  }
  Serial.print(tUs);   Serial.print(',');
  Serial.print(c.gx);  Serial.print(',');
  Serial.print(c.gy);  Serial.print(',');
  Serial.print(c.gz);  Serial.print(',');
  Serial.print(c.ax);  Serial.print(',');
  Serial.print(c.ay);  Serial.print(',');
  Serial.println(c.az);
}

static int puttScore(uint32_t durationMs, uint32_t backswingMs, uint32_t forwardMs) {
  int score = 0;
  float tempo = forwardMs > 0 ? (float)backswingMs / (float)forwardMs : 0.0f;
  float axisQuality = swing.axisQualitySamples > 0 ? swing.axisQualitySum / swing.axisQualitySamples : 0.0f;
  float linearAvg = swing.linearSamples > 0 ? swing.linearSumMps2 / swing.linearSamples : 0.0f;

  if (durationMs >= 180 && durationMs <= 1800) {
    score += 2;
  } else if (durationMs >= 100 && durationMs <= 2400) {
    score += 1;
  }

  if (swing.preStillMs >= READY_MIN_HOLD_MS) {
    score += 2;
  } else if (swing.preStillMs >= MIN_ADDRESS_STILL_MS) {
    score += 1;
  }

  if (swing.maxGyroDps >= 18.0f) {
    score += 2;
  } else if (swing.maxGyroDps >= MIN_PUTT_GYRO_DPS) {
    score += 1;
  }

  if (swing.maxLinearMps2 >= START_LINEAR_MPS2 || linearAvg >= PRETRIGGER_LINEAR_MPS2) {
    score += 1;
  }

  if (swing.transitionDetected && backswingMs >= MIN_BACKSWING_MS && forwardMs >= MIN_FORWARD_MS) {
    score += 2;
  }

  if (tempo >= 0.25f && tempo <= 4.5f) {
    score += 2;
  } else if (tempo >= MIN_TEMPO_RATIO && tempo <= MAX_TEMPO_RATIO) {
    score += 1;
  }

  if (swing.maxBackGyroDps >= MIN_BACK_GYRO_DPS && swing.maxForwardGyroDps >= MIN_FORWARD_GYRO_DPS) {
    score += 2;
  }

  if (swing.backImpulseDeg >= MIN_BACK_IMPULSE_DEG && swing.forwardImpulseDeg >= MIN_FORWARD_IMPULSE_DEG) {
    score += 1;
  }

  if (axisQuality >= 0.35f) {
    score += 2;
  } else if (axisQuality >= MIN_AXIS_QUALITY) {
    score += 1;
  }

  if (swing.directionChanges <= 4) {
    score += 1;
  } else if (swing.directionChanges > MAX_DIRECTION_CHANGES) {
    score -= 2;
  }

  return score;
}

static bool hasStrokeLikeEnvelope(uint32_t durationMs) {
  float axisQuality = swing.axisQualitySamples > 0 ? swing.axisQualitySum / swing.axisQualitySamples : 0.0f;

  return durationMs >= 300 &&
         durationMs <= 1800 &&
         swing.preStillMs >= READY_MIN_HOLD_MS &&
         (swing.maxGyroDps >= MIN_PUTT_GYRO_DPS || swing.maxLinearMps2 >= MIN_STROKE_LINEAR_MPS2) &&
         axisQuality >= MIN_AXIS_QUALITY &&
         swing.directionChanges <= 8;
}

static void finishSwing(uint32_t nowMs) {
  // The LEGACY impact-gated detector now OWNS the result UI. Once a result is on
  // screen (RESULT_HOLD) we must not re-detect or clobber it; the FSM stays in
  // RESULT_HOLD until the user dismisses it.
  g_lastFinishAccepted = false;
  if (state == SENSOR_RESULT_HOLD) {
    resetBuffer();
    return;
  }
  uint32_t durationMs = nowMs - swing.startMs;
  uint32_t backswingMs = swing.transitionDetected ? swing.transitionMs - swing.startMs : 0;
  uint32_t forwardMs = forwardMsFor(nowMs);
  int score = puttScore(durationMs, backswingMs, forwardMs);
  const char *rejectReason = nullptr;
  // Manual mode: the user pressed START and deliberately putt, so accept any
  // structurally-valid stroke. The strict gates (motion/score/impact) exist only
  // to suppress AUTO-mode false positives.
  bool manual = (appMode == MODE_MANUAL);

  if (durationMs < MIN_PUTT_DURATION_MS) {
    rejectReason = "too_short";
  } else if (durationMs > MAX_PUTT_DURATION_MS) {
    rejectReason = "too_long";
  } else if (swing.samples < MIN_PUTT_SAMPLES) {
    rejectReason = "too_few_samples";
  } else if (!manual && armedCaptureEvaluation &&
             swing.maxGyroDps < ARMED_MIN_GYRO_DPS &&
             swing.maxLinearMps2 < ARMED_MIN_LINEAR_MPS2) {
    rejectReason = "no_motion";
  } else if (!manual && !armedCaptureEvaluation &&
             swing.maxGyroDps < MIN_PUTT_GYRO_DPS &&
             swing.maxLinearMps2 < START_LINEAR_MPS2) {
    rejectReason = "weak_motion";
  } else if (swing.maxLinearMps2 > MAX_PUTT_LINEAR_MPS2) {
    rejectReason = "too_much_linear";
  } else if (!manual && !armedCaptureEvaluation &&
             (score < MIN_PUTT_SCORE || (!swing.transitionDetected && !hasStrokeLikeEnvelope(durationMs)))) {
    rejectReason = "low_score";
  } else if (!manual && !swing.impactDetected) {
    rejectReason = "no_impact";  // AUTO only: a putt requires a stroke AND an impact
  }

  if (rejectReason == nullptr) {
    lastResult.durationMs = durationMs;
    lastResult.backswingMs = backswingMs;
    lastResult.forwardMs = forwardMs;
    lastResult.tempo = (float)backswingMs / (float)forwardMs;
    lastResult.maxGyroDps = swing.maxGyroDps;
    lastResult.maxForwardGyroDps = swing.maxForwardGyroDps;
    lastResult.maxLinearMps2 = swing.maxLinearMps2;
    lastResult.maxAccelDeltaMps2 = swing.maxAccelDeltaMps2;
    lastResult.score = score;
    lastResult.impactDetected = swing.impactDetected;
    lastResult.impactOffsetMs = swing.impactDetected ? (int32_t)(swing.impactMs - swing.startMs) : -1;
    lastResult.faceAngleImpactDeg = swing.faceAngleImpactCaptured ? swing.faceAngleImpactDeg : swing.faceAngleDeg;
    lastResult.faceAngleImpactCaptured = swing.faceAngleImpactCaptured;
    lastResult.pathAngleImpactDeg = swing.pathAngleImpactDeg;

    // Legacy detector accepted a stroke. Signal the caller so it can build the
    // UI result from the OrientationTracker over this stroke window. The FSM is
    // recycled to SETTLING here; the result-building path moves it to
    // RESULT_HOLD afterward (which suppresses re-detection).
    g_lastFinishAccepted = true;
    printResult("PUTT_DETECTED", nullptr, nowMs);
    logRawStrokeRecent(swing.startMs, nowMs);
    state = SENSOR_SETTLING;
    stateSinceMs = nowMs;
    requireStillnessForReady = false;
    fsmArmed = false;
  } else {
    printResult("PUTT_REJECTED", rejectReason, nowMs);
    state = SENSOR_SETTLING;
    stateSinceMs = nowMs;
    requireStillnessForReady = false;
    fsmArmed = false;
  }

  resetBuffer();
}

static void startSwing(const Vec3 &gyroRad, uint32_t triggerMs) {
  uint32_t startMs = bufferedSwingStartMs(triggerMs);
  resetSwing(startMs);
  zeroFaceAngleAt(triggerMs);
  swing.gyroAxis = normalize3(gyroRad);
  Vec3 lateral = cross3(gravityReady ? gravityAxis : Vec3{0.0f, 0.0f, 1.0f}, swing.gyroAxis);
  if (mag3(lateral) < 0.0001f) {
    swing.lateralAxis = perpendicular3(swing.gyroAxis);
  } else {
    swing.lateralAxis = normalize3(lateral);
  }
  swing.lastMotionMs = triggerMs;
  swing.traceLastMs = triggerMs;
  swing.maxGyroDps = fabsf(dot3(gyroRad, swing.gyroAxis) * RAD_TO_DEG_F);
  swing.axisAccum = gyroRad;
  swing.preStillMs = triggerMs - stateSinceMs;

  // Track orientation across the live stroke from the address pose so the trace
  // (headPoint) and face/path (decompose) come from the OrientationTracker, the
  // same way the buffered path does. Gravity at address approximates the shaft.
  g_orient.begin(gravityReady ? gravityAxis : Vec3{0.0f, 0.0f, 1.0f});
  g_orientActive = true;
  orientLastMs = triggerMs;

  state = SENSOR_SWING;
  stateSinceMs = startMs;
  Serial.print(F("SWING_STARTED,ms="));
  Serial.print(startMs);
  Serial.print(F(",trigger_ms="));
  Serial.print(triggerMs);
  Serial.print(F(",gyro_dps="));
  Serial.println(swing.maxGyroDps, 1);

  if (ENABLE_DEBUG_STATUS) {
    Serial.print(F("CANDIDATE_STARTED,ms="));
    Serial.print(startMs);
    Serial.print(F(",trigger_ms="));
    Serial.print(triggerMs);
    Serial.print(F(",pretrigger_ms="));
    Serial.print(triggerMs - startMs);
    Serial.print(F(",gyro_dps="));
    Serial.print(swing.maxGyroDps, 1);
    Serial.print(F(",address_still_ms="));
    Serial.println(swing.preStillMs);
  }
}

static void resetToReadyAfterInvalid(uint32_t nowMs) {
  resetBuffer();
  state = SENSOR_SETTLING;
  stateSinceMs = nowMs;
  requireStillnessForReady = false;
  fsmArmed = false;
}

static void updateReady(uint32_t nowMs, float dps, const Vec3 &gyroRad, const Vec3 &linearAccel, const Vec3 &accelMps2, float linearMps2) {
  if (state == SENSOR_SETTLING) {
    if (requireStillnessForReady) {
      if (dps < READY_STILL_GYRO_DPS && linearMps2 < READY_STILL_LINEAR_MPS2) {
        if (nowMs - stateSinceMs >= BOOT_READY_HOLD_MS) {
          enterIdle(nowMs);  // auto: arm & listen; manual: drop to home
        }
      } else {
        stateSinceMs = nowMs;
      }
    } else if (nowMs - stateSinceMs >= POST_PUTT_COOLDOWN_MS &&
               dps < REARM_MAX_GYRO_DPS &&
               linearMps2 < READY_STILL_LINEAR_MPS2) {
      enterIdle(nowMs);
    }
    return;
  }

  if (state == SENSOR_READY) {
    pushBuffer(nowMs, gyroRad, linearAccel, accelMps2, dps, linearMps2);
    if (dps > readyPeakGyroDps) {
      readyPeakGyroDps = dps;
    }

    bool isStill = dps < READY_STILL_GYRO_DPS && linearMps2 < READY_STILL_LINEAR_MPS2;
    if (!fsmArmed) {
      if (isStill) {
        if (readyStillSinceMs == 0) {
          readyStillSinceMs = nowMs;
        }
        if (nowMs - readyStillSinceMs >= FSM_ARM_STILL_MS) {
          fsmArmed = true;
          readyStartCount = 0;
          Serial.print(F("AUTO_ARMED,ms="));
          Serial.println(nowMs);
        }
      } else {
        readyStillSinceMs = 0;
        readyStartCount = 0;
      }
      return;
    }

    if (isStill) {
      readyStartCount = 0;
      return;
    }

    if (dps >= FSM_START_GYRO_DPS && linearMps2 < MAX_START_LINEAR_MPS2) {
      if (readyStartCount < START_CONFIRM_SAMPLES) {
        readyStartCount++;
      }
      if (ENABLE_DEBUG_STATUS) {
        Serial.print(F("START_CANDIDATE,ms="));
        Serial.print(nowMs);
        Serial.print(F(",gyro_dps="));
        Serial.print(dps, 1);
        Serial.print(F(",count="));
        Serial.print(readyStartCount);
        Serial.print(F(",need="));
        Serial.println(START_CONFIRM_SAMPLES);
      }
      if (readyStartCount >= START_CONFIRM_SAMPLES) {
        startSwing(gyroRad, nowMs);
      }
    } else if (dps < PRETRIGGER_RESET_GYRO_DPS) {
      readyStartCount = 0;
    }
  }
}

static void recordHeadTrace(uint32_t nowMs, const Vec3 &gyroRad, float projectionDps) {
  uint32_t deltaMs = nowMs - swing.traceLastMs;
  if (deltaMs == 0 || deltaMs < TRACE_POINT_MIN_MS) {
    return;
  }
  if (deltaMs > 80) {
    swing.traceLastMs = nowMs;
    return;
  }
  swing.traceLastMs = nowMs;

  // Real clubhead path from the orientation tracker: head hangs HEAD_PATH_RADIUS
  // (normalized shaft length) below the butt-mounted sensor; project its swept
  // position into the address horizontal plane.
  HeadPoint hp = g_orient.headPoint(swing.gyroAxis, HEAD_PATH_RADIUS);
  pushTrace(hp.x, hp.y);
}

static void updateFaceAngleEstimate(uint32_t nowMs, const Vec3 &gyroRad) {
  uint32_t deltaMs = nowMs - swing.faceLastMs;
  swing.faceLastMs = nowMs;
  if (deltaMs == 0 || deltaMs > 80) {
    return;
  }

  Vec3 faceReferenceAxis = gravityReady ? gravityAxis : Vec3{0.0f, 0.0f, 1.0f};
  float yawDps = dot3(gyroRad, faceReferenceAxis) * RAD_TO_DEG_F;
  swing.faceAngleDeg += yawDps * ((float)deltaMs / 1000.0f);
}

static float updateSwingAxis(uint32_t nowMs, const Vec3 &gyroRad, float dps) {
  if (!swing.transitionDetected && nowMs - swing.startMs <= AXIS_LEARN_MS && dps > PRETRIGGER_GYRO_DPS) {
    Vec3 axisSample = gyroRad;
    if (dot3(axisSample, swing.gyroAxis) < 0.0f) {
      axisSample = scale3(axisSample, -1.0f);
    }
    swing.axisAccum = add3(swing.axisAccum, axisSample);
    if (mag3(swing.axisAccum) > 0.0001f) {
      swing.gyroAxis = normalize3(swing.axisAccum);
    }
  }

  return dot3(gyroRad, swing.gyroAxis) * RAD_TO_DEG_F;
}

static void updateSwingFeatures(uint32_t nowMs, float dps, const Vec3 &gyroRad, float linearMps2, float projectionDps) {
  uint32_t deltaMs = nowMs - swing.traceLastMs;
  float dt = (deltaMs > 0 && deltaMs <= 80) ? (float)deltaMs / 1000.0f : 0.0f;
  float absProjectionDps = fabsf(projectionDps);

  updateFaceAngleEstimate(nowMs, gyroRad);

  if (dps > 0.1f) {
    swing.axisQualitySum += absProjectionDps / dps;
    swing.axisQualitySamples++;
  }

  swing.linearSumMps2 += linearMps2;
  swing.linearSamples++;
  if (linearMps2 > swing.maxLinearMps2) {
    swing.maxLinearMps2 = linearMps2;
  }

  int8_t direction = 0;
  if (projectionDps > GYRO_REVERSAL_DPS) {
    direction = 1;
  } else if (projectionDps < -GYRO_REVERSAL_DPS) {
    direction = -1;
  }
  if (direction != 0) {
    if (swing.lastDirection != 0 && direction != swing.lastDirection) {
      swing.directionChanges++;
    }
    swing.lastDirection = direction;
  }

  if (dt <= 0.0f) {
    return;
  }

  if (!swing.transitionDetected) {
    if (projectionDps > 0.0f) {
      swing.backImpulseDeg += projectionDps * dt;
      if (projectionDps > swing.maxBackGyroDps) {
        swing.maxBackGyroDps = projectionDps;
      }
    }
  } else if (projectionDps < 0.0f) {
    float forwardGyroDps = -projectionDps;
    swing.forwardImpulseDeg += forwardGyroDps * dt;
  }
}

static void updateSwing(uint32_t nowMs, float dps, const Vec3 &gyroRad, const Vec3 &linearAccel, float linearMps2) {
  if (state != SENSOR_SWING) {
    return;
  }

  swing.samples++;
  if (dps >= STOP_GYRO_DPS) {
    swing.lastMotionMs = nowMs;
  }

  float projectionDps = updateSwingAxis(nowMs, gyroRad, dps);
  updateSwingFeatures(nowMs, dps, gyroRad, linearMps2, projectionDps);

  // Advance the OrientationTracker BEFORE plotting the head / capturing impact so
  // both reflect this sample's pose (mirrors processBufferedSample).
  if (g_orientActive) {
    float odt = (float)(nowMs - orientLastMs) / 1000.0f;
    orientLastMs = nowMs;
    if (odt > 0.0f && odt < 0.1f) {
      g_orient.integrate(gyroRad, odt);
    }
  }

  if (swing.previousLinearAccelReady) {
    float accelDelta = mag3(sub3(linearAccel, swing.previousLinearAccelMps2));
    if (accelDelta > swing.maxAccelDeltaMps2) {
      swing.maxAccelDeltaMps2 = accelDelta;
    }
    if (!swing.impactDetected && swing.transitionDetected && accelDelta >= IMPACT_ACCEL_DELTA_MPS2) {
      swing.impactDetected = true;
      swing.impactMs = nowMs;
      // Face/path from the OrientationTracker at impact (sign convention: open=R,
      // closed=L), matching the buffered path. faceAngleDeg kept as a fallback.
      StrokeAngles ang = g_orient.decompose(swing.gyroAxis);
      swing.faceAngleImpactDeg = -ang.faceDeg;
      swing.pathAngleImpactDeg = ang.pathDeg;
      swing.faceAngleImpactCaptured = true;
      traceImpactIndex = traceCount > 0 ? traceCount - 1 : 0;
      Serial.print(F("IMPACT_DETECTED,ms="));
      Serial.print(nowMs);
      Serial.print(F(",accel_delta_mps2="));
      Serial.print(accelDelta, 2);
      Serial.print(F(",face_angle_deg="));
      Serial.println(swing.faceAngleImpactDeg, 1);
    }
  }
  swing.previousLinearAccelMps2 = linearAccel;
  swing.previousLinearAccelReady = true;
  recordHeadTrace(nowMs, gyroRad, projectionDps);
  if (fabsf(projectionDps) > swing.maxGyroDps) {
    swing.maxGyroDps = fabsf(projectionDps);
  }

  if (!swing.transitionDetected &&
      nowMs - swing.startMs >= MIN_BACKSWING_MS &&
      swing.backImpulseDeg >= MIN_BACK_IMPULSE_DEG &&
      projectionDps < -GYRO_REVERSAL_DPS) {
    swing.transitionDetected = true;
    swing.transitionMs = nowMs;
    traceTransitionIndex = traceCount;
    Serial.print(F("FORWARD_STROKE,ms="));
    Serial.println(nowMs);
  }

  if (swing.transitionDetected && projectionDps < 0.0f) {
    float forwardGyroDps = -projectionDps;
    if (forwardGyroDps > swing.maxForwardGyroDps) {
      swing.maxForwardGyroDps = forwardGyroDps;
    }

    float endThreshold = swing.maxForwardGyroDps * FORWARD_END_FRACTION;
    if (endThreshold < STOP_GYRO_DPS) {
      endThreshold = STOP_GYRO_DPS;
    }

    if (!swing.forwardEndDetected &&
        swing.maxForwardGyroDps >= MIN_PUTT_GYRO_DPS &&
        nowMs - swing.transitionMs >= MIN_FORWARD_MS &&
        forwardGyroDps <= endThreshold) {
      swing.forwardEndDetected = true;
      swing.forwardEndMs = nowMs;
    }
  }

  if ((swing.forwardEndDetected && nowMs - swing.forwardEndMs > POST_FORWARD_END_HOLD_MS) ||
      (nowMs - swing.startMs > MAX_PUTT_DURATION_MS) ||
      (dps < STOP_GYRO_DPS && nowMs - swing.lastMotionMs > SWING_STOP_HOLD_MS)) {
    if (!swing.transitionDetected && nowMs - swing.startMs < 900) {
      Serial.print(F("PUTT_ABORTED,reason=no_forward,ms="));
      Serial.println(nowMs);
      resetToReadyAfterInvalid(nowMs);
      return;
    }
    finishSwing(nowMs);
    if (g_lastFinishAccepted) {
      // Real-time accept (buffered path didn't fire first): surface a result
      // from the live trace + legacy estimated angles so it isn't lost.
      buildLegacyUiResultRealtime(nowMs);
    }
  }
}

// Manual-mode countdown timing only. All home/result TOUCH interaction is now
// handled by LVGL widget events (ui_on_click); this just advances the countdown
// state machine and arms when it hits zero. The on-screen digit is driven by
// ui_sync_from_state() -> ui_apply().
static void updateHomeAndCountdown(uint32_t nowMs) {
  if (state == SENSOR_COUNTDOWN) {
    uint32_t elapsed = nowMs - countdownStartMs;
    if (elapsed >= MANUAL_COUNTDOWN_MS) {
      armForSwing(nowMs);  // -> SENSOR_READY (manual: armed & listening)
    }
  }
}

static void printStatus(uint32_t nowMs, float dps) {
  if (!ENABLE_DEBUG_STATUS) {
    return;
  }

  if (nowMs - lastStatusMs < 500 || state == SENSOR_SETTLING || state == SENSOR_RESULT_HOLD) {
    return;
  }

  lastStatusMs = nowMs;
  Serial.print(F("DETECTOR,state="));
  if (state == SENSOR_READY) {
    Serial.print(F("ready"));
  } else {
    Serial.print(F("swing"));
  }
  Serial.print(F(",gyro_dps="));
  Serial.print(dps, 1);
  if (state == SENSOR_READY) {
    Serial.print(F(",start_threshold_dps="));
    Serial.print(START_GYRO_DPS, 1);
    Serial.print(F(",ready_peak_gyro_dps="));
    Serial.print(readyPeakGyroDps, 1);
    Serial.print(F(",buffer_samples="));
    Serial.print(sampleBufferCount);
  }
  Serial.println();
}

static void printNoImuStatus(uint32_t nowMs) {
  Serial.print(F("STATUS,ms="));
  Serial.print(nowMs);
  Serial.print(F(",state=no_imu,fw="));
  Serial.print(FW_VERSION);
  Serial.println(F(",hint=check_board_selection_or_imu_i2c"));
}

void app_report_on_serial_connect(uint32_t nowMs) {
  bool serialConnected = Serial;
  if (serialConnected && !serialWasConnected) {
    if (state == SENSOR_READY) {
      printReadyEvent(nowMs);
    } else if (state == SENSOR_NO_IMU) {
      printNoImuStatus(nowMs);
    }
  }
  serialWasConnected = serialConnected;
}

// ---------------------------------------------------------------------------
// Public app_* entry points (declared in app_fsm.h). These are the ONLY symbols
// that cross into main.ino; everything above is private to this translation
// unit. They wrap the FSM/detector internals so setup()/loop() stay thin.
// ---------------------------------------------------------------------------

// FSM/calibration/detector init. Installs the widget click handler and seeds the
// home-face selector + status timers. Called from setup() after ui_init().
void app_init() {
  // Route widget clicks (set in ui_screens.cpp) to our flag-only handler.
  ui_click_fn = ui_on_click;
  g_uiHomeAuto = (appMode == MODE_AUTO);  // boot mode -> initial home face
  lastStatusMs = millis();
  lastImuRetryMs = millis();
}

// The entire !imuReady loop branch: heartbeat LED, periodic no-IMU status,
// periodic IMU re-probe, and keep the (NO IMU) screen alive. Returns after a
// short delay, exactly as the old loop branch did.
void app_no_imu_tick(uint32_t nowMs) {
  digitalWrite(LED_BUILTIN, (nowMs / 250) % 2);
  if (nowMs - lastStatusMs >= 1000) {
    lastStatusMs = nowMs;
    printNoImuStatus(nowMs);
  }
  if (nowMs - lastImuRetryMs >= 3000) {
    lastImuRetryMs = nowMs;
    app_start_imu_session();
  }
  // Keep the UI alive (NO IMU screen) and process deferred swaps.
  if (ui_ready()) {
    ui_sync_from_state(nowMs);
    ui_apply();
  }
  delay(20);
}

// IMU sample-rate gate. Returns true (and advances the schedule) only when at
// least SAMPLE_PERIOD_US has elapsed since the previous accepted sample.
bool app_sample_due(uint32_t nowUs) {
  if ((uint32_t)(nowUs - lastSampleUs) < SAMPLE_PERIOD_US) {
    return false;
  }
  lastSampleUs = nowUs;
  return true;
}

// Called by loop() when readImu() fails: drop back into the no-IMU retry path.
void app_on_imu_read_failed() {
  imuReady = false;
  state = SENSOR_NO_IMU;
}

// One per-loop tick for a freshly-read sample: raw-CSV logging, the g_putt
// PUTTV2 diagnostic, gravity/bias calibration, the legacy FSM detection chain,
// the manual countdown, status printing, the UI sync, and the activity LED.
// Mirrors the old loop() body after readImu() exactly.
void app_update(uint32_t nowMs, uint32_t nowUs, const ImuSample& sample, const RawImuCounts& rawCounts) {
  logRawCsvSample(nowUs, rawCounts);

  PuttEvent pe = g_putt.update(RawSample{nowUs, rawCounts.gx, rawCounts.gy, rawCounts.gz,
                                         rawCounts.ax, rawCounts.ay, rawCounts.az});
  if (pe.detected) {
    Serial.print(pe.decision == PuttDecision::Accept ? F("PUTTV2_ACCEPT,reason=ok")
                                                     : F("PUTTV2_REJECT,reason="));
    if (pe.decision != PuttDecision::Accept) Serial.print(pe.reason ? pe.reason : "");
    Serial.print(F(",jerk="));    Serial.print(pe.features.impactJerk, 1);
    Serial.print(F(",fwd_dps=")); Serial.print(pe.features.peakForwardDps, 1);
    Serial.print(F(",axis="));    Serial.print(pe.features.axisConsistency, 2);
    Serial.print(F(",dur_ms="));  Serial.println(pe.features.durationMs);

    // The LEGACY impact-gated detector now OWNS the result UI (it is more
    // sensitive). g_putt keeps running for the PUTTV2 serial diagnostics above
    // and side-by-side comparison, but it NO LONGER drives the result screen.
  }

  float rawDps = gyroDps(sample.gyroRad);
  Vec3 gyro = filteredGyroSample(sample.gyroRad);
  float dps = gyroDps(gyro);
  Vec3 linearAccel = linearAccelMps2(sample.accelMps2);
  float linearMps2 = mag3(linearAccel);

  if ((state == SENSOR_SETTLING || state == SENSOR_READY) &&
      rawDps < STILL_GYRO_DPS) {
    updateGravityEstimate(sample.accelMps2);
  }

  if (state == SENSOR_SETTLING && requireStillnessForReady && rawDps < STILL_GYRO_DPS) {
    updateGyroBias(sample.gyroRad);
  }

  updateReady(nowMs, dps, gyro, linearAccel, sample.accelMps2, linearMps2);
  if (state == SENSOR_SWING) {
    pushBuffer(nowMs, gyro, linearAccel, sample.accelMps2, dps, linearMps2);
  }
  updateSwing(nowMs, dps, gyro, linearAccel, linearMps2);
  updateHomeAndCountdown(nowMs);
  printStatus(nowMs, dps);

  // Drive the LVGL UI from the app FSM, then apply any deferred screen swap.
  // ui_apply() is the ONLY place screens are loaded/deleted (never in events).
  ui_sync_from_state(nowMs);
  ui_apply();

  digitalWrite(LED_BUILTIN,
               state == SENSOR_READY ||
               state == SENSOR_SWING ||
               state == SENSOR_RESULT_HOLD ? HIGH : LOW);
}
