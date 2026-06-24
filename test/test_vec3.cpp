#include "check.h"
#include "vec3.h"

static void test_mag_dot() {
  Vec3 a{3, 4, 0};
  CHECK_NEAR(mag3(a), 5.0f, 1e-4);
  Vec3 b{1, 0, 0}, c{0, 1, 0};
  CHECK_NEAR(dot3(b, c), 0.0f, 1e-6);
  CHECK_NEAR(dot3(b, b), 1.0f, 1e-6);
}

static void test_normalize_project() {
  Vec3 v{0, 0, 5};
  Vec3 n = normalize3(v);
  CHECK_NEAR(mag3(n), 1.0f, 1e-5);
  // projection of (1,2,3) onto unit z is (0,0,3)
  Vec3 p = scale3(n, dot3(Vec3{1, 2, 3}, n));
  CHECK_NEAR(p.z, 3.0f, 1e-5);
  CHECK_NEAR(p.x, 0.0f, 1e-5);
}

int main() {
  RUN(test_mag_dot);
  RUN(test_normalize_project);
  REPORT();
}
