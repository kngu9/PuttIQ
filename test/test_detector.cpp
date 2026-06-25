#include "check.h"
#include "putt_detector.h"
#include <cmath>

static const int16_t ONE_G = (int16_t)(1.0f / 0.000122f);
static int16_t dpsToCount(float dps) { return (int16_t)(dps / 0.0175f); }
static int16_t mps2ToCount(float a)  { return (int16_t)(a / (0.000122f * 9.80665f)); }

// Single-axis pendulum on X; optional one-sample lateral-accel spike at the
// forward peak to emulate a ball-impact jerk. Returns # of accepted detections.
static int feed(PuttDetector &det, bool withImpact) {
  int acc = 0; uint32_t t = 0;
  auto step = [&](float dpsX, float linX) {
    PuttEvent e = det.update(RawSample{t, dpsToCount(dpsX), 0, 0,
                                       mps2ToCount(linX), 0, ONE_G});
    if (e.detected && e.decision == PuttDecision::Accept) ++acc;
    t += 5000;
  };
  for (int i = 0; i < 200; ++i) step(0, 0);                       // settle + pre-still
  for (int i = 0; i < 30; ++i)  step(-25 * std::sin(M_PI*i/30), 0); // backswing
  for (int i = 0; i < 30; ++i) {                                   // forward
    float lin = (withImpact && i == 15) ? 25.0f : 0.0f;           // impact spike
    step(70 * std::sin(M_PI*i/30), lin);
  }
  for (int i = 0; i < 120; ++i) step(0, 0);                       // settle + post window
  return acc;
}

static void test_detects_putt_with_impact() {
  PuttDetector det{DetectorConfig{}};
  CHECK(feed(det, true) == 1);
}

static void test_no_impact_no_detection() {
  PuttDetector det{DetectorConfig{}};
  CHECK(feed(det, false) == 0);   // no jerk spike -> nothing triggers
}

static void test_static_knock_rejected() {
  // A jerk spike with NO rotation: triggers, but rejected (no stroke).
  PuttDetector det{DetectorConfig{}};
  int acc = 0; uint32_t t = 0;
  auto step = [&](float linX) {
    PuttEvent e = det.update(RawSample{t, 0, 0, 0, mps2ToCount(linX), 0, ONE_G});
    if (e.detected && e.decision == PuttDecision::Accept) ++acc;
    t += 5000;
  };
  for (int i = 0; i < 200; ++i) step(0);
  step(25.0f);                    // knock, no rotation
  for (int i = 0; i < 120; ++i) step(0);
  CHECK(acc == 0);
}

int main() {
  RUN(test_detects_putt_with_impact);
  RUN(test_no_impact_no_detection);
  RUN(test_static_knock_rejected);
  REPORT();
}
