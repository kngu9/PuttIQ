#pragma once
#include <cstdint>

struct Vec3 { float x, y, z; };

// Exactly what the hardware/log row provides — raw register counts.
struct RawSample {
  uint32_t tUs;            // microsecond timestamp
  int16_t gx, gy, gz;      // gyro raw counts (±500 dps full scale)
  int16_t ax, ay, az;      // accel raw counts (±4 g full scale)
};

// Output of the preprocessing pipeline.
struct DerivedSample {
  uint32_t tUs;
  Vec3 gyroRad;            // bias-corrected, low-pass filtered, rad/s
  float gyroDps;           // |gyroRad| in deg/s
  Vec3 linearMps2;         // gravity-compensated acceleration, m/s^2
  float linearMag;         // |linearMps2|
  bool valid;              // true once gyro-bias + gravity have settled
};

struct PuttFeatures {
  uint32_t durationMs;
  float axisConsistency;   // 0..1, energy fraction on dominant axis
  int   reversalCount;     // clean direction changes about dominant axis
  float peakForwardDps;    // peak signed projection on forward direction
  float bellRatio;         // peak/mean of forward-stroke speed (bell-ness)
  float linAngRatio;       // linear energy / angular energy
  float preStillMs;        // quiet time before the candidate
  float postSettleMs;      // quiet time after the candidate
  float peakLinearMag;     // peak |linear accel| in window
  bool  impactPresent;     // a jerk spike near the forward peak
  float impactJerk;        // magnitude of the largest jerk spike
};

enum class PuttDecision { Accept, Reject };

struct DetectorConfig {
  // Preprocessing
  float gyroBiasAlpha   = 0.015f;
  float gyroFilterAlpha = 0.45f;
  float gravityAlpha    = 0.04f;
  float gravityMinMps2  = 7.0f;
  float gravityMaxMps2  = 12.5f;
  float biasStillDps    = 6.0f;    // freeze bias adaptation above this motion
  // Candidate windowing
  float candidateRiseDps = 70.0f;   // motion floor that opens a candidate
  float candidateQuietDps = 2.0f;  // below this = quiet
  uint32_t candidateQuietHoldMs = 150;  // quiet this long = candidate closed
  uint32_t maxGapUs = 20000;       // sample gap that invalidates a candidate
  // Decision gates (tuned against corpus later)
  uint32_t minDurationMs = 80;
  uint32_t maxDurationMs = 3000;
  float minAxisConsistency = 0.55f;
  int   maxReversals = 3;
  float maxLinAngRatio = 0.6f;
  float minPeakForwardDps = 60.0f;
  float minBellRatio = 1.2f;  // a clean half-sine forward stroke is ~1.4; 1.5 rejected all smooth strokes
  uint32_t minPreStillMs = 0;     // disabled: real use is never fully still
  uint32_t minPostSettleMs = 0;   // disabled: motion continues after the stroke
  uint32_t maxWindowMs = 1000;    // force-decide a candidate after this long
  bool  requireImpact = false;     // flipped on later if data supports it
  float impactJerkMps3 = 3.4f;     // jerk threshold (m/s^2 between samples)
  // Impact-triggered windowing
  uint32_t windowPreMs  = 600;   // capture this long BEFORE the impact
  uint32_t windowPostMs = 400;   // and this long AFTER (contact is late in the stroke)
  uint32_t impactRefractoryMs = 800;  // ignore new impacts this long after one
};

struct PuttEvent {
  bool detected;
  PuttDecision decision;
  const char* reason;     // why rejected (nullptr if accepted)
  PuttFeatures features;
};

static const int TRACE_CAP = 96;   // max downsampled head-trace points

struct TraceXY { float x, y; };

// Everything the UI needs from a decided stroke, computed in the pure detector.
struct PuttResult {
  bool detected;
  PuttDecision decision;
  const char* reason;
  // metrics
  float faceDeg;          // clubface twist at impact (open +, closed -)
  float pathDeg;          // swing-plane tilt (in/out)
  float tempo;            // backswing:forward ratio
  uint32_t backswingMs;
  uint32_t forwardMs;
  uint32_t durationMs;
  float peakForwardDps;
  bool  impactPresent;
  float impactJerk;
  // clubhead trace (address frame, downsampled), impact at impactIndex
  uint16_t traceCount;
  TraceXY trace[TRACE_CAP];
  uint16_t impactIndex;
};
