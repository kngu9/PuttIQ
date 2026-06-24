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
  // Candidate windowing
  float candidateRiseDps = 4.0f;   // motion floor that opens a candidate
  float candidateQuietDps = 2.0f;  // below this = quiet
  uint32_t candidateQuietHoldMs = 150;  // quiet this long = candidate closed
  uint32_t maxGapUs = 20000;       // sample gap that invalidates a candidate
  // Decision gates (tuned against corpus later)
  uint32_t minDurationMs = 80;
  uint32_t maxDurationMs = 3000;
  float minAxisConsistency = 0.55f;
  int   maxReversals = 3;
  float maxLinAngRatio = 0.6f;
  float minPeakForwardDps = 8.0f;
  float minBellRatio = 1.5f;
  uint32_t minPreStillMs = 120;
  uint32_t minPostSettleMs = 120;
  bool  requireImpact = false;     // flipped on later if data supports it
  float impactJerkMps3 = 3.4f;     // jerk threshold (m/s^2 between samples)
};

struct PuttEvent {
  bool detected;
  PuttDecision decision;
  const char* reason;     // why rejected (nullptr if accepted)
  PuttFeatures features;
};
