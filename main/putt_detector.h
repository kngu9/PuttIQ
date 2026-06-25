#pragma once
#include "imu_types.h"
#include "putt_preprocess.h"

class PuttDetector {
public:
  explicit PuttDetector(const DetectorConfig &cfg)
    : cfg_(cfg), pre_(cfg) {}
  void reset();
  // Feed one raw sample; returns an event with detected=true only on the
  // sample that closes (and decides) a candidate window.
  PuttEvent update(const RawSample &raw);

  // Full consolidated result (trace + metrics) for the most recently decided
  // window. Valid after update() returns an event with detected==true.
  PuttResult result() const { return result_; }

private:
  static const int CAP = 1024;            // ~5 s at 200 Hz
  PuttEvent decide();                     // run features+gates on current window
  void buildResult(const PuttEvent &ev);  // populate result_ from the window

  DetectorConfig cfg_;
  Preprocessor pre_;
  DerivedSample buf_[CAP];
  int count_ = 0;                         // valid samples held (capped)
  bool inCandidate_ = false;
  int candStart_ = 0;                     // index in buf_ where motion opened
  uint32_t lastQuietStartUs_ = 0;
  bool quietRunning_ = false;
  uint32_t lastTUs_ = 0;
  bool haveLast_ = false;
  Vec3 windowGravity_{0, 0, 1};           // gravity axis captured when candidate opened
  PuttResult result_{};                   // cached result of last decided window
};
