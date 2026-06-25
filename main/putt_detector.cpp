#include "putt_detector.h"
#include "putt_features.h"
#include <cmath>

void PuttDetector::reset() {
  pre_.reset();
  count_ = 0; inCandidate_ = false; candStart_ = 0;
  quietRunning_ = false; haveLast_ = false;
}

PuttEvent PuttDetector::update(const RawSample &raw) {
  PuttEvent none{}; none.detected = false; none.reason = nullptr;

  // Gap detection: a large time gap invalidates any in-progress candidate.
  if (haveLast_ && (raw.tUs - lastTUs_) > cfg_.maxGapUs) {
    inCandidate_ = false; count_ = 0;
  }
  lastTUs_ = raw.tUs; haveLast_ = true;

  DerivedSample d = pre_.process(raw);
  if (!d.valid) return none;  // still settling

  // Shift buffer if full (keep most recent CAP samples).
  if (count_ >= CAP) {
    for (int i = 1; i < CAP; ++i) buf_[i-1] = buf_[i];
    --count_;
    if (candStart_ > 0) --candStart_; else inCandidate_ = false;
  }
  buf_[count_++] = d;

  bool quiet = d.gyroDps < cfg_.candidateQuietDps;

  if (!inCandidate_) {
    if (d.gyroDps >= cfg_.candidateRiseDps) {
      inCandidate_ = true;
      // Start a few samples back to capture pre-still + ramp.
      candStart_ = count_ - 1;
      int back = 0;
      while (candStart_ > 0 && back < 40) { --candStart_; ++back; }
      quietRunning_ = false;
    }
    return none;
  }

  // In a candidate: track a sustained quiet run that closes the window.
  if (quiet) {
    if (!quietRunning_) { quietRunning_ = true; lastQuietStartUs_ = d.tUs; }
    else if ((d.tUs - lastQuietStartUs_) >= cfg_.candidateQuietHoldMs * 1000UL) {
      PuttEvent ev = decide();
      inCandidate_ = false; quietRunning_ = false;
      // Drop decided samples so the next stroke starts clean.
      count_ = 0;
      return ev;
    }
  } else {
    quietRunning_ = false;
  }
  return none;
}

PuttEvent PuttDetector::decide() {
  PuttEvent ev{}; ev.detected = true;
  int n = count_ - candStart_;
  ev.features = extractFeatures(&buf_[candStart_], n);
  PuttFeatures &f = ev.features;
  f.impactPresent = f.impactJerk >= cfg_.impactJerkMps3;

  auto reject = [&](const char *why) {
    ev.decision = PuttDecision::Reject; ev.reason = why; return ev;
  };

  if (f.durationMs < cfg_.minDurationMs) return reject("too_short");
  if (f.durationMs > cfg_.maxDurationMs) return reject("too_long");
  if (f.axisConsistency < cfg_.minAxisConsistency) return reject("multi_axis");
  if (f.reversalCount > cfg_.maxReversals) return reject("too_many_reversals");
  if (f.linAngRatio > cfg_.maxLinAngRatio) return reject("too_much_linear");
  if (f.peakForwardDps < cfg_.minPeakForwardDps) return reject("weak_motion");
  if (f.bellRatio < cfg_.minBellRatio) return reject("not_bell");
  if (f.preStillMs < cfg_.minPreStillMs) return reject("no_address");
  if (f.postSettleMs < cfg_.minPostSettleMs) return reject("no_settle");
  if (cfg_.requireImpact && !f.impactPresent) return reject("no_impact");

  ev.decision = PuttDecision::Accept; ev.reason = nullptr;
  return ev;
}
