#pragma once
#include "imu_types.h"

// samples: contiguous window including quiet margins on both ends.
PuttFeatures extractFeatures(const DerivedSample *samples, int n);

// Geometry of a decided window: the dominant swing axis, the active span
// (first..last sample above the quiet floor), the reversal sample (backswing ->
// forward sign change on-axis), and the forward-lobe peak sample. Shared with
// extractFeatures so trace/tempo use the same axis + lobe split as the gates.
struct WindowGeometry {
  Vec3 axis;
  int  reversalIdx;     // sample where on-axis projection crosses to forward sign
  int  forwardPeakIdx;  // sample of the forward-lobe peak
  int  activeStart;     // first sample above quiet floor
  int  activeEnd;       // last sample above quiet floor
};
WindowGeometry analyzeWindowGeometry(const DerivedSample *samples, int n);
