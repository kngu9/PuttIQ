#include "putt_features.h"
#include "vec3.h"
#include <cmath>

static const float RAD_TO_DEG = 57.2957795f;

static const float AXIS_MIN_DPS = 1.0f; // ignore near-still samples when finding the axis

// Dominant rotation axis = principal direction of summed |gyro| vectors
// (sign-aligned so lobes don't cancel).
static Vec3 dominantAxis(const DerivedSample *s, int n) {
  Vec3 acc{0, 0, 0};
  Vec3 ref{1, 0, 0};
  for (int i = 0; i < n; ++i) {
    if (s[i].gyroDps < AXIS_MIN_DPS) continue;
    Vec3 g = s[i].gyroRad;
    if (dot3(g, ref) < 0.0f) g = scale3(g, -1.0f);
    acc = add3(acc, g);
    if (mag3(acc) > 1e-4f) ref = normalize3(acc);
  }
  return normalize3(acc);
}

// Intentionally above DetectorConfig.candidateQuietDps (2 dps): margins/duration
// require a firmer stillness than the detector's candidate windowing.
static const float QUIET_DPS = 5.0f;

WindowGeometry analyzeWindowGeometry(const DerivedSample *s, int n) {
  WindowGeometry g{};
  g.axis = {1, 0, 0};
  g.reversalIdx = g.forwardPeakIdx = g.activeStart = g.activeEnd = 0;
  if (n < 2) return g;

  Vec3 axis = dominantAxis(s, n);
  g.axis = axis;

  // Active span (first..last sample above QUIET).
  int a = 0; while (a < n && s[a].gyroDps < QUIET_DPS) ++a;
  int b = n - 1; while (b > a && s[b].gyroDps < QUIET_DPS) --b;
  g.activeStart = a;
  g.activeEnd = b;

  // Signed projection peaks within the active span, with their indices.
  float peakPos = 0, peakNeg = 0; int idxPos = -1, idxNeg = -1;
  for (int i = a; i <= b && a <= b; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG;
    if (proj > peakPos) { peakPos = proj; idxPos = i; }
    if (proj < peakNeg) { peakNeg = proj; idxNeg = i; }
  }
  // Forward = the lobe occurring LATER in time.
  int forwardSign = (idxPos >= idxNeg) ? 1 : -1;
  g.forwardPeakIdx = (forwardSign > 0) ? idxPos : idxNeg;
  if (g.forwardPeakIdx < 0) g.forwardPeakIdx = a;

  // Reversal = first sample where the on-axis projection crosses from the
  // backswing sign to the forward sign within the active span.
  g.reversalIdx = a;
  for (int i = a; i <= b && a <= b; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG;
    if ((float)forwardSign * proj > 0.0f) { g.reversalIdx = i; break; }
  }
  return g;
}

PuttFeatures extractFeatures(const DerivedSample *s, int n) {
  PuttFeatures f{};
  if (n < 2) return f;

  Vec3 axis = dominantAxis(s, n);

  // Energy on-axis vs off-axis, and linear vs angular.
  double onAxis = 0, angEnergy = 0, linEnergy = 0;
  float peakLinear = 0;
  for (int i = 0; i < n; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG; // dps on axis
    float magd = s[i].gyroDps;
    onAxis += (double)proj * proj;
    angEnergy += (double)magd * magd;
    linEnergy += (double)s[i].linearMag * s[i].linearMag;
    if (s[i].linearMag > peakLinear) peakLinear = s[i].linearMag;
  }
  f.axisConsistency = angEnergy > 1e-6 ? (float)(onAxis / angEnergy) : 0.0f;
  // Deliberately mixes (m/s^2)^2 with (dps)^2 -> unitless-by-convention; only
  // meaningful relative to the corpus-tuned maxLinAngRatio (do not "fix" units).
  f.linAngRatio = angEnergy > 1e-6 ? (float)(linEnergy / angEnergy) : 0.0f;
  f.peakLinearMag = peakLinear;

  // Active span (first..last sample above QUIET). Compute BEFORE the lobe analysis.
  int a = 0; while (a < n && s[a].gyroDps < QUIET_DPS) ++a;
  int b = n - 1; while (b > a && s[b].gyroDps < QUIET_DPS) --b;
  f.durationMs = (b > a) ? (s[b].tUs - s[a].tUs) / 1000 : 0;

  // Signed projection peaks within the active span, with their indices.
  float peakPos = 0, peakNeg = 0; int idxPos = -1, idxNeg = -1;
  for (int i = a; i <= b && a <= b; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG;
    if (proj > peakPos) { peakPos = proj; idxPos = i; }
    if (proj < peakNeg) { peakNeg = proj; idxNeg = i; }
  }
  // Forward = the lobe occurring LATER in time.
  int forwardSign = (idxPos >= idxNeg) ? 1 : -1;
  int forwardPeakIdx = (forwardSign > 0) ? idxPos : idxNeg;
  f.peakForwardDps = (forwardSign > 0) ? peakPos : -peakNeg; // magnitude of forward lobe

  // Reversal count on the dominant axis, within the active span, with hysteresis.
  const float REV_BAND_DPS = 4.0f;  // dps band to count a reversal
  int reversals = 0, dir = 0;
  for (int i = a; i <= b && a <= b; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG;
    int d = (proj > REV_BAND_DPS) ? 1 : (proj < -REV_BAND_DPS) ? -1 : 0;
    if (d != 0 && d != dir) { if (dir != 0) ++reversals; dir = d; }
  }
  f.reversalCount = reversals;

  // Bell-ness of the forward lobe only: peak / mean over samples in the active
  // span whose signed projection has the forward sign and exceeds 20% of peak.
  float thr = 0.2f * f.peakForwardDps;
  double fsum = 0; int fcount = 0;
  for (int i = a; i <= b && a <= b; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG * forwardSign; // >0 inside fwd lobe
    if (proj >= thr && proj > 0) { fsum += proj; ++fcount; }
  }
  float mean = fcount ? (float)(fsum / fcount) : 0.0f;
  f.bellRatio = mean > 1e-3f ? f.peakForwardDps / mean : 0.0f;

  // Quiet margins: leading/trailing run below QUIET_DPS. Guard < n so an
  // all-quiet window (lead==n / trail==n) does not read s[n] or s[-1].
  int lead = 0; while (lead < n && s[lead].gyroDps < QUIET_DPS) ++lead;
  int trail = 0; while (trail < n && s[n-1-trail].gyroDps < QUIET_DPS) ++trail;
  f.preStillMs   = (lead  > 0 && lead  < n) ? (s[lead].tUs - s[0].tUs) / 1000.0f : 0.0f;
  f.postSettleMs = (trail > 0 && trail < n) ? (s[n-1].tUs - s[n-1-trail].tUs) / 1000.0f : 0.0f;

  // Impact: largest jerk (delta linear accel between consecutive samples),
  // searched from the forward peak onward (avoids pre-stroke handling bumps).
  float maxJerk = 0;
  int start = (forwardPeakIdx > 0) ? forwardPeakIdx : 1;
  for (int i = start; i < n; ++i) {
    if (i < 1) continue;
    float jerk = mag3(sub3(s[i].linearMps2, s[i-1].linearMps2));
    if (jerk > maxJerk) maxJerk = jerk;
  }
  f.impactJerk = maxJerk;
  f.impactPresent = false;  // decided against config threshold in the detector

  return f;
}
