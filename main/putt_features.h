#pragma once
#include "imu_types.h"

// samples: contiguous window including quiet margins on both ends.
PuttFeatures extractFeatures(const DerivedSample *samples, int n);
