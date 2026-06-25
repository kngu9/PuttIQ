#pragma once
#include "imu_types.h"
#include "putt_preprocess.h"

// Impact-triggered detector: a free-running ring buffer; when a jerk spike
// (impact) is seen, it captures an asymmetric window around the impact (more
// before than after, since contact is at the end of the forward stroke) and
// emits it as a stroke if the window contains a single-axis rotation.
class PuttDetector {
public:
  explicit PuttDetector(const DetectorConfig &cfg) : cfg_(cfg), pre_(cfg) {}
  void reset();
  // Feed one raw sample. detected=true only on the sample that finishes
  // collecting a window after an impact (then decision = Accept/Reject).
  PuttEvent update(const RawSample &raw);

private:
  static const int CAP = 384;            // ~2 s of history
  PuttEvent decideWindow(uint32_t impactTUs);

  DetectorConfig cfg_;
  Preprocessor pre_;
  DerivedSample buf_[CAP];               // ring buffer
  DerivedSample window_[CAP];            // scratch for the extracted window
  int head_ = 0;                         // next write slot
  int count_ = 0;                        // valid samples held (<= CAP)
  bool havePrev_ = false;
  DerivedSample prev_{};                 // previous valid sample (for jerk)
  uint32_t lastTUs_ = 0;
  bool haveLast_ = false;
  bool collecting_ = false;
  uint32_t impactTUs_ = 0;
  uint32_t refractoryUntilTUs_ = 0;
  bool haveRefractory_ = false;
};
