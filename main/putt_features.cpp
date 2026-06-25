#include "putt_features.h"
#include "vec3.h"
#include <cmath>

static const float RAD_TO_DEG = 57.2957795f;

// Dominant rotation axis = principal direction of summed |gyro| vectors
// (sign-aligned so lobes don't cancel).
static Vec3 dominantAxis(const DerivedSample *s, int n) {
  Vec3 acc{0, 0, 0};
  Vec3 ref{1, 0, 0};
  for (int i = 0; i < n; ++i) {
    if (s[i].gyroDps < 1.0f) continue;
    Vec3 g = s[i].gyroRad;
    if (dot3(g, ref) < 0.0f) g = scale3(g, -1.0f);
    acc = add3(acc, g);
    if (mag3(acc) > 1e-4f) ref = normalize3(acc);
  }
  return normalize3(acc);
}

PuttFeatures extractFeatures(const DerivedSample *s, int n) {
  PuttFeatures f{};
  if (n < 2) return f;

  Vec3 axis = dominantAxis(s, n);

  // Energy on-axis vs off-axis, and linear vs angular.
  double onAxis = 0, total = 0, angEnergy = 0, linEnergy = 0;
  float peakLinear = 0;
  for (int i = 0; i < n; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG; // dps on axis
    float magd = s[i].gyroDps;
    onAxis += (double)proj * proj;
    total  += (double)magd * magd;
    angEnergy += (double)magd * magd;
    linEnergy += (double)s[i].linearMag * s[i].linearMag;
    if (s[i].linearMag > peakLinear) peakLinear = s[i].linearMag;
  }
  f.axisConsistency = total > 1e-6 ? (float)(onAxis / total) : 0.0f;
  f.linAngRatio = angEnergy > 1e-6 ? (float)(linEnergy / angEnergy) : 0.0f;
  f.peakLinearMag = peakLinear;

  // Reversal count + signed peak forward, on the dominant axis, with hysteresis.
  const float REV = 4.0f;  // dps band to count a reversal
  int reversals = 0, dir = 0;
  float peakPos = 0, peakNeg = 0;
  for (int i = 0; i < n; ++i) {
    float proj = dot3(s[i].gyroRad, axis) * RAD_TO_DEG;
    if (proj > peakPos) peakPos = proj;
    if (proj < peakNeg) peakNeg = proj;
    int d = (proj > REV) ? 1 : (proj < -REV) ? -1 : 0;
    if (d != 0 && d != dir) { if (dir != 0) ++reversals; dir = d; }
  }
  f.reversalCount = reversals;
  // "Forward" = the larger-magnitude lobe.
  f.peakForwardDps = (peakPos >= -peakNeg) ? peakPos : -peakNeg;

  // Bell-ness of the forward lobe: peak / mean over samples above 20% of peak.
  float thr = 0.2f * f.peakForwardDps;
  double fsum = 0; int fcount = 0;
  for (int i = 0; i < n; ++i) {
    float proj = std::fabs(dot3(s[i].gyroRad, axis) * RAD_TO_DEG);
    if (proj >= thr) { fsum += proj; ++fcount; }
  }
  float mean = fcount ? (float)(fsum / fcount) : 0.0f;
  f.bellRatio = mean > 1e-3f ? f.peakForwardDps / mean : 0.0f;

  // Quiet margins: leading/trailing run below candidateQuietDps proxy (5 dps).
  const float QUIET = 5.0f;
  int lead = 0; while (lead < n && s[lead].gyroDps < QUIET) ++lead;
  int trail = 0; while (trail < n && s[n-1-trail].gyroDps < QUIET) ++trail;
  f.preStillMs   = lead  > 0 ? (s[lead].tUs - s[0].tUs) / 1000.0f : 0.0f;
  f.postSettleMs = trail > 0 ? (s[n-1].tUs - s[n-1-trail].tUs) / 1000.0f : 0.0f;

  // Active span = first to last sample above QUIET.
  int a = 0; while (a < n && s[a].gyroDps < QUIET) ++a;
  int b = n - 1; while (b > a && s[b].gyroDps < QUIET) --b;
  f.durationMs = (b > a) ? (s[b].tUs - s[a].tUs) / 1000 : 0;

  // Impact: largest jerk (delta linear accel between consecutive samples),
  // searched from the forward peak onward.
  float maxJerk = 0;
  for (int i = 1; i < n; ++i) {
    float jerk = mag3(sub3(s[i].linearMps2, s[i-1].linearMps2));
    if (jerk > maxJerk) maxJerk = jerk;
  }
  f.impactJerk = maxJerk;
  f.impactPresent = false;  // decided against config threshold in the detector

  return f;
}
