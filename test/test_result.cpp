#include "check.h"
#include "putt_detector.h"
#include <cmath>
#include <cstdio>

static const int16_t ONE_G = (int16_t)(1.0f / 0.000122f);

// Feed a synthetic single-axis pendulum putt as RAW samples (same style as
// test_detector.cpp): settle, backswing, forward with an impact jerk, settle.
// Returns true on a detected event and fills out 'res' with the result.
static bool feedPutt(PuttDetector &det, PuttResult &res) {
  bool got = false;
  uint32_t t = 0;
  auto step = [&](float dpsX, int16_t axImpact) {
    int16_t gx = (int16_t)(dpsX / 0.0175f);
    PuttEvent e = det.update(RawSample{t, gx, 0, 0, axImpact, 0, ONE_G});
    if (e.detected) { got = true; res = det.result(); }
    t += 5000;
  };
  for (int i = 0; i < 200; ++i) step(0, 0);                          // settle + pre-still
  for (int i = 0; i < 30; ++i) step(-60*std::sin(M_PI*i/30), 0);     // backswing
  for (int i = 0; i < 30; ++i) {
    // forward (strong), with a sharp lateral jerk at the bottom = impact.
    // 30000 raw counts ~= 36 m/s^2 (stays within the int16 +/-4g range).
    int16_t ax = (i == 20) ? (int16_t)30000 : 0;
    step(180*std::sin(M_PI*i/30), ax);
  }
  for (int i = 0; i < 80; ++i) step(0, 0);                           // settle (closes window)
  return got;
}

static void test_result_metrics() {
  PuttDetector det{DetectorConfig{}};
  PuttResult r{};
  bool detected = feedPutt(det, r);

  CHECK(detected);
  CHECK(r.detected);
  CHECK(r.decision == PuttDecision::Accept);

  // Trace: enough points and a clear dominant sweep direction.
  CHECK(r.traceCount > 10);
  float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
  for (int i = 0; i < r.traceCount; ++i) {
    if (r.trace[i].x < xmin) xmin = r.trace[i].x;
    if (r.trace[i].x > xmax) xmax = r.trace[i].x;
    if (r.trace[i].y < ymin) ymin = r.trace[i].y;
    if (r.trace[i].y > ymax) ymax = r.trace[i].y;
  }
  float xrange = xmax - xmin, yrange = ymax - ymin;
  CHECK(xrange > 0.0f || yrange > 0.0f);
  CHECK((xrange > 0.05f) || (yrange > 0.05f));   // a real sweep, not noise
  CHECK(r.impactIndex < r.traceCount);

  // Pure single-axis stroke -> ~zero face twist.
  CHECK_NEAR(r.faceDeg, 0.0f, 3.0f);

  // Tempo: both phases positive and a plausible ratio.
  CHECK(r.backswingMs > 0);
  CHECK(r.forwardMs > 0);
  CHECK(r.tempo > 0.0f);
  CHECK(r.tempo < 10.0f);

  // Impact jerk spike was injected.
  CHECK(r.impactPresent);

  std::printf("   face=%.2f path=%.2f tempo=%.2f back=%ums fwd=%ums dur=%ums "
              "traceCount=%u impactIdx=%u xrange=%.3f yrange=%.3f jerk=%.2f\n",
              r.faceDeg, r.pathDeg, r.tempo, r.backswingMs, r.forwardMs,
              r.durationMs, r.traceCount, r.impactIndex, xrange, yrange,
              r.impactJerk);
}

int main() {
  RUN(test_result_metrics);
  REPORT();
}
