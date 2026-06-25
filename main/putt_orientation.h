#pragma once
#include "imu_types.h"

// Putting metrics from orientation tracking, all in the address frame (degrees).
struct StrokeAngles {
  float faceDeg;   // twist about the shaft axis (open/closed)
  float swingDeg;  // rotation in the stroke plane (the pendulum)
  float pathDeg;   // swing-plane tilt vs the target line (in-to-out/out-to-in)
};

// Integrates body-frame gyro into an orientation quaternion from an address
// reference, then decomposes the accumulated rotation onto an orthonormal
// address basis (shaft / swing / path).
class OrientationTracker {
public:
  // gravityAtAddress: gravity unit vector in the body frame at address
  //   (approximates "down the shaft" -> the face-twist axis).
  // Resets the quaternion to identity; stores the shaft (face-twist) axis.
  void begin(const Vec3 &gravityAtAddress);
  void integrate(const Vec3 &gyroRad, float dt);   // gyro in rad/s, dt in seconds
  // swingAxis: dominant rotation axis of the stroke (body frame), used to
  //   orient the basis azimuth. Need not be perpendicular to the shaft. The
  //   stroke's swing axis is only known partway through the stroke, so the
  //   swing/path basis is built here rather than at begin().
  StrokeAngles decompose(const Vec3 &swingAxis) const;

private:
  float qw_ = 1, qx_ = 0, qy_ = 0, qz_ = 0;        // orientation from address
  Vec3 eShaft_{0,0,1};                             // face-twist axis (address frame)
};
