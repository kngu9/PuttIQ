#include "putt_detector.h"
#include "putt_features.h"
#include "putt_orientation.h"
#include "vec3.h"
#include <cmath>

void PuttDetector::reset() {
  pre_.reset();
  count_ = 0; inCandidate_ = false; candStart_ = 0;
  quietRunning_ = false; haveLast_ = false;
  windowGravity_ = Vec3{0, 0, 1};
  result_ = PuttResult{};
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

  // Force-decide if a candidate has run for the max window (busy environments
  // where motion never settles to quiet would otherwise never close it).
  if (inCandidate_ && (d.tUs - buf_[candStart_].tUs) >= cfg_.maxWindowMs * 1000UL) {
    PuttEvent ev = decide();
    inCandidate_ = false; quietRunning_ = false; count_ = 0;
    return ev;
  }

  if (!inCandidate_) {
    if (d.gyroDps >= cfg_.candidateRiseDps) {
      inCandidate_ = true;
      // Start a few samples back to capture pre-still + ramp.
      candStart_ = count_ - 1;
      int back = 0;
      while (candStart_ > 0 && back < 40) { --candStart_; ++back; }
      quietRunning_ = false;
      // Capture gravity at the moment the candidate opens (the address frame).
      windowGravity_ = pre_.gravityReady() ? pre_.gravityAxis() : Vec3{0, 0, 1};
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
    ev.decision = PuttDecision::Reject; ev.reason = why;
    buildResult(ev);
    return ev;
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
  buildResult(ev);
  return ev;
}

// Consolidate trace + metrics for the current window into result_. Operates on
// the same buffered window decide() just analyzed (buf_[candStart_..count_]).
void PuttDetector::buildResult(const PuttEvent &ev) {
  PuttResult r{};
  r.detected = ev.detected;
  r.decision = ev.decision;
  r.reason = ev.reason;
  r.peakForwardDps = ev.features.peakForwardDps;
  r.impactPresent = ev.features.impactPresent;
  r.impactJerk = ev.features.impactJerk;
  r.durationMs = ev.features.durationMs;

  const DerivedSample *s = &buf_[candStart_];
  int n = count_ - candStart_;
  if (n < 2) { result_ = r; return; }

  WindowGeometry g = analyzeWindowGeometry(s, n);
  const Vec3 axis = g.axis;

  // Tempo: backswing (activeStart..reversal) vs forward (reversal..activeEnd).
  uint32_t tStart = s[g.activeStart].tUs;
  uint32_t tRev   = s[g.reversalIdx].tUs;
  uint32_t tEnd   = s[g.activeEnd].tUs;
  r.backswingMs = (tRev > tStart) ? (tRev - tStart) / 1000 : 0;
  r.forwardMs   = (tEnd > tRev)   ? (tEnd - tRev) / 1000   : 0;
  r.tempo = (float)r.backswingMs / (float)(r.forwardMs > 0 ? r.forwardMs : 1);

  // Orientation up to the forward peak -> face/path at impact.
  OrientationTracker ot;
  ot.begin(windowGravity_);
  for (int i = 1; i <= g.forwardPeakIdx && i < n; ++i) {
    float dt = (s[i].tUs - s[i-1].tUs) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) ot.integrate(s[i].gyroRad, dt);
  }
  StrokeAngles ang = ot.decompose(axis);
  r.faceDeg = -ang.faceDeg;   // firmware sign convention: open=R/positive
  r.pathDeg = ang.pathDeg;

  // Trace: integrate across the full window, sampling head position, then
  // downsample to <= TRACE_CAP points. Impact = nearest point to forwardPeak.
  int stride = (n + TRACE_CAP - 1) / TRACE_CAP;   // ceil(n / TRACE_CAP) >= 1
  if (stride < 1) stride = 1;
  OrientationTracker ot2;
  ot2.begin(windowGravity_);
  int outCount = 0;
  int impactOut = 0;
  for (int i = 0; i < n && outCount < TRACE_CAP; ++i) {
    if (i > 0) {
      float dt = (s[i].tUs - s[i-1].tUs) * 1e-6f;
      if (dt > 0.0f && dt < 0.1f) ot2.integrate(s[i].gyroRad, dt);
    }
    if (i % stride == 0) {
      HeadPoint hp = ot2.headPoint(axis, 1.0f);
      r.trace[outCount].x = hp.x;
      r.trace[outCount].y = hp.y;
      if (i <= g.forwardPeakIdx) impactOut = outCount;  // last point at/before impact
      ++outCount;
    }
  }
  r.traceCount = (uint16_t)outCount;
  r.impactIndex = (uint16_t)(impactOut < outCount ? impactOut : (outCount > 0 ? outCount - 1 : 0));

  result_ = r;
}
