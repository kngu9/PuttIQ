#include "check.h"
#include "putt_detector.h"
#include <cmath>

static const int16_t ONE_G = (int16_t)(1.0f / 0.000122f);

// Feed a synthetic single-axis pendulum (with a small impact bump) as RAW samples.
static int feedPutt(PuttDetector &det) {
  int detections = 0;
  uint32_t t = 0;
  auto step = [&](float dpsX) {
    int16_t gx = (int16_t)(dpsX / 0.0175f);
    PuttEvent e = det.update(RawSample{t, gx, 0, 0, 0, 0, ONE_G});
    if (e.detected && e.decision == PuttDecision::Accept) ++detections;
    t += 5000;
  };
  for (int i = 0; i < 200; ++i) step(0);                     // settle filters + pre-still
  for (int i = 0; i < 30; ++i) step(-25*std::sin(M_PI*i/30));// backswing
  for (int i = 0; i < 30; ++i) step(70*std::sin(M_PI*i/30)); // forward
  for (int i = 0; i < 80; ++i) step(0);                      // settle (closes candidate)
  return detections;
}

// Walking never settles → many reversals / multi-axis → must NOT detect.
static int feedWalk(PuttDetector &det) {
  int detections = 0;
  uint32_t t = 0;
  for (int i = 0; i < 200; ++i) {
    float a = 35*std::sin(2*M_PI*i/12), b = 30*std::sin(2*M_PI*i/9 + 1.0);
    int16_t gx = (int16_t)(a / 0.0175f), gy = (int16_t)(b / 0.0175f);
    PuttEvent e = det.update(RawSample{t, gx, gy, 0, 1500, 1200, ONE_G});
    if (e.detected && e.decision == PuttDecision::Accept) ++detections;
    t += 5000;
  }
  return detections;
}

static void test_detects_putt() {
  PuttDetector det{DetectorConfig{}};
  CHECK(feedPutt(det) == 1);
}

static void test_rejects_walk() {
  PuttDetector det{DetectorConfig{}};
  CHECK(feedWalk(det) == 0);
}

int main() {
  RUN(test_detects_putt);
  RUN(test_rejects_walk);
  REPORT();
}
