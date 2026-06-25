#include "putt_preprocess.h"
#include "vec3.h"

static const float GYRO_DPS_PER_LSB = 0.0175f;
static const float ACCEL_G_PER_LSB  = 0.000122f;
static const float DEG_TO_RAD       = 0.0174532925f;
static const float RAD_TO_DEG       = 57.2957795f;
static const float G                = 9.80665f;

void Preprocessor::reset() {
  gyroBias_ = {0, 0, 0};
  filteredGyro_ = {0, 0, 0};
  gravityAxis_ = {0, 0, 1};
  biasReady_ = gyroFilterReady_ = gravityReady_ = false;
}

DerivedSample Preprocessor::process(const RawSample &raw) {
  Vec3 gyroRaw{ raw.gx * GYRO_DPS_PER_LSB * DEG_TO_RAD,
                raw.gy * GYRO_DPS_PER_LSB * DEG_TO_RAD,
                raw.gz * GYRO_DPS_PER_LSB * DEG_TO_RAD };
  Vec3 accel{ raw.ax * ACCEL_G_PER_LSB * G,
              raw.ay * ACCEL_G_PER_LSB * G,
              raw.az * ACCEL_G_PER_LSB * G };

  // Gyro bias (slow EMA toward the resting value).
  if (!biasReady_) { gyroBias_ = gyroRaw; biasReady_ = true; }
  else gyroBias_ = add3(scale3(gyroBias_, 1.0f - cfg_.gyroBiasAlpha),
                        scale3(gyroRaw, cfg_.gyroBiasAlpha));

  // Bias-corrected + low-pass filtered gyro.
  Vec3 corrected = sub3(gyroRaw, gyroBias_);
  if (!gyroFilterReady_) { filteredGyro_ = corrected; gyroFilterReady_ = true; }
  else filteredGyro_ = add3(scale3(filteredGyro_, 1.0f - cfg_.gyroFilterAlpha),
                            scale3(corrected, cfg_.gyroFilterAlpha));

  // Gravity estimate (only when |accel| looks like ~1 g).
  float accelMag = mag3(accel);
  if (accelMag >= cfg_.gravityMinMps2 && accelMag <= cfg_.gravityMaxMps2) {
    Vec3 axis = normalize3(accel);
    if (!gravityReady_) { gravityAxis_ = axis; gravityReady_ = true; }
    else gravityAxis_ = normalize3(add3(scale3(gravityAxis_, 1.0f - cfg_.gravityAlpha),
                                        scale3(axis, cfg_.gravityAlpha)));
  }

  Vec3 linear = gravityReady_
      ? sub3(accel, scale3(gravityAxis_, dot3(accel, gravityAxis_)))
      : Vec3{0, 0, 0};

  DerivedSample d;
  d.tUs = raw.tUs;
  d.gyroRad = filteredGyro_;
  d.gyroDps = mag3(filteredGyro_) * RAD_TO_DEG;
  d.linearMps2 = linear;
  d.linearMag = mag3(linear);
  d.valid = biasReady_ && gravityReady_;
  return d;
}
