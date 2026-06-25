#include "check.h"
#include "putt_features.h"
#include <vector>
#include <cmath>

static const float DEG_TO_RAD = 0.0174532925f;

// Build a single-axis pendulum: back (negative) then forward (positive) lobe on X,
// gravity already removed (linear ~0). dt = 5 ms.
static std::vector<DerivedSample> makePutt() {
  std::vector<DerivedSample> v;
  auto push = [&](uint32_t tUs, float dpsX, float lin) {
    Vec3 g{dpsX * DEG_TO_RAD, 0, 0};
    v.push_back(DerivedSample{tUs, g, std::fabs(dpsX), {lin, 0, 0}, std::fabs(lin), true});
  };
  uint32_t t = 0;
  for (int i = 0; i < 40; ++i) { push(t, 0, 0); t += 5000; }          // pre-still
  for (int i = 0; i < 30; ++i) { push(t, -20*std::sin(M_PI*i/30), 0); t += 5000; } // backswing
  for (int i = 0; i < 30; ++i) { push(t, 60*std::sin(M_PI*i/30), 0); t += 5000; }  // forward (bell)
  for (int i = 0; i < 40; ++i) { push(t, 0, 0); t += 5000; }          // post-settle
  return v;
}

// Walking: multi-axis, many reversals, never settles.
static std::vector<DerivedSample> makeWalk() {
  std::vector<DerivedSample> v;
  uint32_t t = 0;
  for (int i = 0; i < 140; ++i) {
    float a = 30*std::sin(2*M_PI*i/12);
    float b = 25*std::sin(2*M_PI*i/9 + 1.0);
    Vec3 g{a*DEG_TO_RAD, b*DEG_TO_RAD, 0};
    float dps = std::sqrt(a*a + b*b);
    v.push_back(DerivedSample{t, g, dps, {2.0f, 1.5f, 0}, 2.5f, true});
    t += 5000;
  }
  return v;
}

static void test_putt_features() {
  auto p = makePutt();
  PuttFeatures f = extractFeatures(p.data(), (int)p.size());
  CHECK(f.axisConsistency > 0.9f);     // single axis
  CHECK(f.reversalCount <= 2);         // one clean back->forward
  CHECK(f.peakForwardDps > 40.0f);
  CHECK(f.bellRatio > 1.4f);
  CHECK(f.linAngRatio < 0.3f);         // mostly rotation
  CHECK(f.preStillMs > 100.0f);
  CHECK(f.postSettleMs > 100.0f);
}

static void test_walk_features() {
  auto w = makeWalk();
  PuttFeatures f = extractFeatures(w.data(), (int)w.size());
  // Walk must fail at least one strong discriminator:
  CHECK(f.axisConsistency < 0.85f || f.reversalCount > 3 || f.linAngRatio > 0.6f);
}

int main() {
  RUN(test_putt_features);
  RUN(test_walk_features);
  REPORT();
}
