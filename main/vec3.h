#pragma once
#include <cmath>
#include "imu_types.h"

inline float mag3(const Vec3 &v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }
inline float dot3(const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 sub3(const Vec3 &a, const Vec3 &b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 add3(const Vec3 &a, const Vec3 &b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 scale3(const Vec3 &v, float s) { return {v.x*s, v.y*s, v.z*s}; }
inline Vec3 normalize3(const Vec3 &v) {
  float m = mag3(v);
  if (m < 1e-4f) return {1.0f, 0.0f, 0.0f};
  return scale3(v, 1.0f / m);
}
