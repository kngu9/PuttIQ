#include "putt_detector.h"
#include "putt_features.h"
#include "vec3.h"
#include <cmath>

void PuttDetector::reset() {
  pre_.reset();
  head_ = 0; count_ = 0; havePrev_ = false; haveLast_ = false;
  collecting_ = false; haveRefractory_ = false;
}

PuttEvent PuttDetector::update(const RawSample &raw) {
  PuttEvent none{}; none.detected = false; none.reason = nullptr;

  // A large time gap invalidates in-progress state.
  if (haveLast_ && (raw.tUs - lastTUs_) > cfg_.maxGapUs) {
    head_ = 0; count_ = 0; havePrev_ = false; collecting_ = false;
  }
  lastTUs_ = raw.tUs; haveLast_ = true;

  DerivedSample d = pre_.process(raw);
  if (!d.valid) { return none; }  // still settling

  // Push into ring buffer.
  buf_[head_] = d;
  head_ = (head_ + 1) % CAP;
  if (count_ < CAP) ++count_;

  // Jerk vs previous valid sample.
  float jerk = havePrev_ ? mag3(sub3(d.linearMps2, prev_.linearMps2)) : 0.0f;
  prev_ = d; havePrev_ = true;

  if (collecting_) {
    // Wait until we have enough post-impact samples, then decide.
    if (d.tUs - impactTUs_ >= cfg_.windowPostMs * 1000UL) {
      PuttEvent ev = decideWindow(impactTUs_);
      collecting_ = false;
      refractoryUntilTUs_ = impactTUs_ + cfg_.impactRefractoryMs * 1000UL;
      haveRefractory_ = true;
      return ev;
    }
    return none;
  }

  // Not collecting: look for an impact trigger (respecting refractory).
  bool inRefractory = haveRefractory_ &&
                      (int32_t)(d.tUs - refractoryUntilTUs_) < 0;
  if (!inRefractory && jerk >= cfg_.impactJerkMps3) {
    impactTUs_ = d.tUs;
    collecting_ = true;
  }
  return none;
}

PuttEvent PuttDetector::decideWindow(uint32_t impactTUs) {
  PuttEvent ev{}; ev.detected = true;

  uint32_t lo = (impactTUs > cfg_.windowPreMs * 1000UL)
                  ? impactTUs - cfg_.windowPreMs * 1000UL : 0;
  uint32_t hi = impactTUs + cfg_.windowPostMs * 1000UL;

  // Copy the time window out of the ring buffer (oldest->newest, time-ordered).
  int n = 0;
  for (int i = 0; i < count_; ++i) {
    int idx = (head_ - count_ + i + CAP) % CAP;
    const DerivedSample &s = buf_[idx];
    if (s.tUs >= lo && s.tUs <= hi) window_[n++] = s;
  }

  ev.features = extractFeatures(window_, n);
  PuttFeatures &f = ev.features;
  f.impactPresent = true;  // triggered by impact

  auto reject = [&](const char *why) {
    ev.decision = PuttDecision::Reject; ev.reason = why; return ev;
  };

  // Minimal sanity gates: a real stroke has rotation, and a putt is single-axis.
  if (f.peakForwardDps < cfg_.minPeakForwardDps) return reject("no_stroke");
  if (f.axisConsistency < cfg_.minAxisConsistency) return reject("multi_axis");

  ev.decision = PuttDecision::Accept; ev.reason = nullptr;
  return ev;
}
