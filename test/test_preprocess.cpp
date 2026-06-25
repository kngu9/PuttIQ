#include "check.h"
#include "putt_preprocess.h"

// 1 g downward on Z at rest, no rotation. After enough samples gravity settles
// and linear accel → ~0, gyro → ~0.
static RawSample rest(uint32_t tUs) {
  // 1 g = 9.80665 m/s^2; counts = g_value / (0.000122 * 9.80665)
  const int16_t oneG = (int16_t)(1.0f / 0.000122f);  // ~8197 counts
  return RawSample{tUs, 0, 0, 0, 0, 0, oneG};
}

static void test_rest_settles() {
  Preprocessor pp{DetectorConfig{}};
  DerivedSample d{};
  for (int i = 0; i < 400; ++i) d = pp.process(rest(i * 5000));
  CHECK(d.valid);
  CHECK_NEAR(d.gyroDps, 0.0f, 0.5f);
  CHECK_NEAR(d.linearMag, 0.0f, 0.3f);  // gravity removed
}

static void test_gyro_scaling() {
  // Verify the 0.0175 dps/LSB scaling in isolation. A bias-subtracting filter
  // cannot pass a sustained rotation through unattenuated, so freeze the bias
  // (alpha 0) after it seeds to the resting zero and make the low-pass
  // pass-through (alpha 1.0); then neither filter masks the scaling under test.
  DetectorConfig cfg;
  cfg.gyroBiasAlpha = 0.0f;
  cfg.gyroFilterAlpha = 1.0f;
  Preprocessor pp{cfg};
  pp.process(rest(0));  // seeds bias=0 and gravity from a resting sample
  // 100 dps on X: counts = 100 / 0.0175
  const int16_t r = (int16_t)(100.0f / 0.0175f);
  DerivedSample d = pp.process(RawSample{5000, r, 0, 0, 0, 0,
                                         (int16_t)(1.0f / 0.000122f)});
  CHECK_NEAR(d.gyroDps, 100.0f, 1.0f);
}

int main() {
  RUN(test_rest_settles);
  RUN(test_gyro_scaling);
  REPORT();
}
