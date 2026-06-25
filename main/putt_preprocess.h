#pragma once
#include "imu_types.h"

class Preprocessor {
public:
  explicit Preprocessor(const DetectorConfig &cfg) : cfg_(cfg) {}
  void reset();
  DerivedSample process(const RawSample &raw);
  Vec3 gravityAxis() const { return gravityAxis_; }
  bool gravityReady() const { return gravityReady_; }

private:
  DetectorConfig cfg_;
  Vec3 gyroBias_{0, 0, 0};
  Vec3 filteredGyro_{0, 0, 0};
  Vec3 gravityAxis_{0, 0, 1};
  bool biasReady_ = false;
  bool gyroFilterReady_ = false;
  bool gravityReady_ = false;
};
