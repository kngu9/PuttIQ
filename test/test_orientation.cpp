#include "check.h"
#include "putt_orientation.h"
#include "vec3.h"
#include <cmath>

static const float D2R = 0.0174532925f;

// Integrate a constant body-frame angular velocity about unit axis `w` for a
// total rotation of `totalRad`, in many small steps.
static void spin(OrientationTracker &o, Vec3 w, float totalRad) {
  const int N = 400; const float dt = 0.0005f;
  float mag = std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
  float rate = totalRad / (N * dt);             // |omega| so |omega|*N*dt = totalRad
  Vec3 ww{ w.x/mag*rate, w.y/mag*rate, w.z/mag*rate };
  for (int i = 0; i < N; ++i) o.integrate(ww, dt);
}

// Address basis: shaft = +z, swing = +x, so path = shaft x swing = +y.
static void test_pure_face() {
  OrientationTracker o; o.begin(Vec3{0,0,1});
  spin(o, Vec3{0,0,1}, 10*D2R);                 // 10 deg twist about shaft
  StrokeAngles a = o.decompose(Vec3{1,0,0});
  CHECK_NEAR(a.faceDeg, 10.0f, 0.5f);
  CHECK_NEAR(a.swingDeg, 0.0f, 0.5f);
  CHECK_NEAR(a.pathDeg, 0.0f, 0.5f);
}

static void test_pure_swing() {
  OrientationTracker o; o.begin(Vec3{0,0,1});
  spin(o, Vec3{1,0,0}, 30*D2R);                 // 30 deg pendulum about swing axis
  StrokeAngles a = o.decompose(Vec3{1,0,0});
  CHECK_NEAR(a.swingDeg, 30.0f, 0.5f);
  CHECK_NEAR(a.faceDeg, 0.0f, 0.5f);
  CHECK_NEAR(a.pathDeg, 0.0f, 0.5f);
}

static void test_pure_path() {
  OrientationTracker o; o.begin(Vec3{0,0,1});
  spin(o, Vec3{0,1,0}, 8*D2R);                  // 8 deg plane tilt about path axis
  StrokeAngles a = o.decompose(Vec3{1,0,0});
  CHECK_NEAR(a.pathDeg, 8.0f, 0.5f);
  CHECK_NEAR(a.faceDeg, 0.0f, 0.5f);
  CHECK_NEAR(a.swingDeg, 0.0f, 0.5f);
}

// Address basis with a non-axis-aligned shaft still decomposes a pure twist
// about that shaft as pure face.
static void test_tilted_shaft_twist() {
  Vec3 shaft = normalize3(Vec3{0.3f, 0.0f, 1.0f});  // ~17 deg from vertical (lie tilt)
  OrientationTracker o; o.begin(shaft);
  spin(o, shaft, 6*D2R);
  StrokeAngles a = o.decompose(Vec3{1,0,0});
  CHECK_NEAR(a.faceDeg, 6.0f, 0.6f);
  CHECK_NEAR(a.swingDeg, 0.0f, 0.6f);
}

// A pure pendulum swing about the swing axis sweeps the head forward/back (y),
// with negligible lateral (x) motion -> a straight line.
static void test_trace_pendulum_straight() {
  OrientationTracker o; o.begin(Vec3{0,0,1});
  // sample the head at a few rotation amounts about the swing axis (+x)
  HeadPoint p0 = o.headPoint(Vec3{1,0,0}, 1.0f);
  spin(o, Vec3{1,0,0}, 20*D2R);
  HeadPoint p1 = o.headPoint(Vec3{1,0,0}, 1.0f);
  CHECK_NEAR(p0.x, 0.0f, 1e-3); CHECK_NEAR(p0.y, 0.0f, 1e-3);   // origin at address
  CHECK(std::fabs(p1.y) > 0.2f);          // swept forward/back appreciably
  CHECK_NEAR(p1.x, 0.0f, 0.02f);          // negligible lateral -> straight
  // sin(20deg)*L ~ 0.342
  CHECK_NEAR(std::fabs(p1.y), 0.342f, 0.03f);
}

// A path-plane tilt (rotation about ePath = +y here) moves the head laterally (x).
static void test_trace_path_curves() {
  OrientationTracker o; o.begin(Vec3{0,0,1});
  spin(o, Vec3{0,1,0}, 15*D2R);           // tilt about path axis
  HeadPoint p = o.headPoint(Vec3{1,0,0}, 1.0f);
  CHECK(std::fabs(p.x) > 0.2f);           // lateral deflection appears
  // sin(15deg)*L ~ 0.259
  CHECK_NEAR(std::fabs(p.x), 0.259f, 0.03f);
}

int main() {
  RUN(test_pure_face);
  RUN(test_pure_swing);
  RUN(test_pure_path);
  RUN(test_tilted_shaft_twist);
  RUN(test_trace_pendulum_straight);
  RUN(test_trace_path_curves);
  REPORT();
}
