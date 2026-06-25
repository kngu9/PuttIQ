#include "putt_orientation.h"
#include "vec3.h"
#include <cmath>

static const float RAD_TO_DEG = 57.2957795f;

void OrientationTracker::begin(const Vec3 &gravityAtAddress) {
  qw_ = 1; qx_ = 0; qy_ = 0; qz_ = 0;
  eShaft_ = normalize3(gravityAtAddress);
}

void OrientationTracker::integrate(const Vec3 &w, float dt) {
  // q += 0.5 * q (x) (0, w) * dt ; then renormalize
  float dw = -0.5f * (qx_*w.x + qy_*w.y + qz_*w.z) * dt;
  float dx =  0.5f * (qw_*w.x + qy_*w.z - qz_*w.y) * dt;
  float dy =  0.5f * (qw_*w.y - qx_*w.z + qz_*w.x) * dt;
  float dz =  0.5f * (qw_*w.z + qx_*w.y - qy_*w.x) * dt;
  qw_ += dw; qx_ += dx; qy_ += dy; qz_ += dz;
  float m = std::sqrt(qw_*qw_ + qx_*qx_ + qy_*qy_ + qz_*qz_);
  if (m > 1e-9f) { qw_/=m; qx_/=m; qy_/=m; qz_/=m; }
}

StrokeAngles OrientationTracker::decompose(const Vec3 &swingAxis) const {
  // Build the swing/path basis now (the swing axis is only known mid-stroke).
  // Orthogonalize the swing axis against the shaft (Gram-Schmidt).
  Vec3 sw = sub3(swingAxis, scale3(eShaft_, dot3(swingAxis, eShaft_)));
  if (mag3(sw) < 1e-4f) {
    Vec3 t = (std::fabs(eShaft_.x) < 0.9f) ? Vec3{1,0,0} : Vec3{0,1,0};
    sw = sub3(t, scale3(eShaft_, dot3(t, eShaft_)));
  }
  Vec3 eSwing = normalize3(sw);
  Vec3 ePath = cross3(eShaft_, eSwing);   // unit, completes the right-handed basis

  // Convert the quaternion to an axis-angle rotation vector (address frame).
  float w = qw_;
  if (w > 1.0f) w = 1.0f;
  if (w < -1.0f) w = -1.0f;
  float angle = 2.0f * std::acos(w);
  Vec3 r{0,0,0};
  float s = std::sqrt(1.0f - w*w);          // = sin(angle/2)
  if (s > 1e-6f) {
    float k = angle / s;                    // axis = (qx,qy,qz)/s ; r = axis*angle
    r = Vec3{ qx_*k, qy_*k, qz_*k };
  }
  StrokeAngles a;
  a.faceDeg  = dot3(r, eShaft_) * RAD_TO_DEG;
  a.swingDeg = dot3(r, eSwing) * RAD_TO_DEG;
  a.pathDeg  = dot3(r, ePath)  * RAD_TO_DEG;
  return a;
}
