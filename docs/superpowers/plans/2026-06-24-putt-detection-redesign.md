# Putt Detection Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the heuristic, hardware-coupled putt detector with a pure, host-testable, impact-gated envelope detector that eliminates pickup/walk/waggle false positives.

**Architecture:** Detection logic moves out of `main/main.ino` into pure C++ modules (no Arduino deps) that take *raw* IMU samples and own all filtering. A host-side replay harness + unit tests run the exact same modules against recorded labeled clips, so thresholds are tuned and proven offline. `main.ino` becomes thin glue: read IMU → `detector.update(raw)` → emit events. Data capture (CSV over serial) feeds the corpus.

**Tech Stack:** C++ (Arduino for firmware; `g++ -std=c++17` for host tests/replay), Seeed XIAO nRF52840 Sense, LSM6DS3 IMU, plain Makefile + a tiny header-only test framework.

---

## Data Contract (referenced by all tasks)

These types live in `main/imu_types.h` (Task 2) and are the shared vocabulary. Read this before any task.

```cpp
struct Vec3 { float x, y, z; };

// Exactly what the hardware/log row provides — raw register counts.
struct RawSample {
  uint32_t tUs;            // microsecond timestamp
  int16_t gx, gy, gz;      // gyro raw counts (±500 dps full scale)
  int16_t ax, ay, az;      // accel raw counts (±4 g full scale)
};

// Output of the preprocessing pipeline.
struct DerivedSample {
  uint32_t tUs;
  Vec3 gyroRad;            // bias-corrected, low-pass filtered, rad/s
  float gyroDps;           // |gyroRad| in deg/s
  Vec3 linearMps2;         // gravity-compensated acceleration, m/s^2
  float linearMag;         // |linearMps2|
  bool valid;              // true once gyro-bias + gravity have settled
};

struct PuttFeatures {
  uint32_t durationMs;
  float axisConsistency;   // 0..1, energy fraction on dominant axis
  int   reversalCount;     // clean direction changes about dominant axis
  float peakForwardDps;    // peak signed projection on forward direction
  float bellRatio;         // peak/mean of forward-stroke speed (bell-ness)
  float linAngRatio;       // linear energy / angular energy
  float preStillMs;        // quiet time before the candidate
  float postSettleMs;      // quiet time after the candidate
  float peakLinearMag;     // peak |linear accel| in window
  bool  impactPresent;     // a jerk spike near the forward peak
  float impactJerk;        // magnitude of the largest jerk spike
};

enum class PuttDecision { Accept, Reject };

struct DetectorConfig {
  // Preprocessing
  float gyroBiasAlpha   = 0.015f;
  float gyroFilterAlpha = 0.45f;
  float gravityAlpha    = 0.04f;
  float gravityMinMps2  = 7.0f;
  float gravityMaxMps2  = 12.5f;
  // Candidate windowing
  float candidateRiseDps = 4.0f;   // motion floor that opens a candidate
  float candidateQuietDps = 2.0f;  // below this = quiet
  uint32_t candidateQuietHoldMs = 150;  // quiet this long = candidate closed
  uint32_t maxGapUs = 20000;       // sample gap that invalidates a candidate
  // Decision gates (tuned against corpus in Task 18)
  uint32_t minDurationMs = 80;
  uint32_t maxDurationMs = 3000;
  float minAxisConsistency = 0.55f;
  int   maxReversals = 3;
  float maxLinAngRatio = 0.6f;
  float minPeakForwardDps = 8.0f;
  float minBellRatio = 1.5f;
  uint32_t minPreStillMs = 120;
  uint32_t minPostSettleMs = 120;
  bool  requireImpact = false;     // flipped on in Task 18 if data supports it
  float impactJerkMps3 = 3.4f;     // jerk threshold (m/s^2 between samples)
};

struct PuttEvent {
  bool detected;
  PuttDecision decision;
  const char* reason;     // why rejected (nullptr if accepted)
  PuttFeatures features;
};
```

Scaling constants (from current firmware, `main.ino:114-115,1218-1223`):
`GYRO_500DPS_DPS_PER_LSB = 0.0175f`, `ACCEL_4G_G_PER_LSB = 0.000122f`,
`DEG_TO_RAD_F = 0.0174532925f`, `RAD_TO_DEG_F = 57.2957795f`, `G = 9.80665f`.

---

## File Structure

```
main/
  main.ino            # MODIFY: thin glue (IMU read loop + serial output + CSV logging)
  imu_types.h         # CREATE: shared types (Data Contract above)
  vec3.h              # CREATE: header-only Vec3 math (ported from main.ino:282-327)
  putt_preprocess.h   # CREATE: Preprocessor (scaling + bias + gravity + linear)
  putt_preprocess.cpp # CREATE
  putt_features.h     # CREATE: feature extraction over a window
  putt_features.cpp   # CREATE
  putt_detector.h     # CREATE: PuttDetector (buffer + candidate FSM + decision)
  putt_detector.cpp   # CREATE
test/
  check.h             # CREATE: tiny test framework
  test_vec3.cpp       # CREATE
  test_preprocess.cpp # CREATE
  test_features.cpp   # CREATE
  test_detector.cpp   # CREATE
  replay.cpp          # CREATE: host tool, reads a CSV clip → prints events
  fixtures/           # CREATE: recorded labeled clips (.csv)
Makefile              # CREATE: host build for tests + replay
```

`vec3.h`, `imu_types.h`, and all `putt_*` files use only `<cstdint>`/`<cmath>` — they compile unchanged on both host and device. The detector uses **fixed-size arrays only** (no STL) so it is embedded-safe.

---

## Task 1: Host build + test framework

**Files:**
- Create: `test/check.h`
- Create: `Makefile`

- [ ] **Step 1: Write the test framework header**

Create `test/check.h`:

```cpp
#pragma once
#include <cstdio>
#include <cmath>
#include <cstdlib>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond) do { \
  ++g_checks; \
  if (!(cond)) { ++g_failures; \
    std::printf("FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
  ++g_checks; \
  double _d = std::fabs((double)(a) - (double)(b)); \
  if (_d > (tol)) { ++g_failures; \
    std::printf("FAIL %s:%d  CHECK_NEAR(%s=%g, %s=%g, tol=%g) diff=%g\n", \
      __FILE__, __LINE__, #a, (double)(a), #b, (double)(b), (double)(tol), _d); } \
} while (0)

#define RUN(fn) do { std::printf("-- %s\n", #fn); fn(); } while (0)

#define REPORT() do { \
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures); \
  return g_failures == 0 ? 0 : 1; \
} while (0)
```

- [ ] **Step 2: Write the Makefile**

Create `Makefile`:

```make
CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Imain -Itest

TESTS := test_vec3 test_preprocess test_features test_detector

.PHONY: test replay clean
test: $(TESTS:%=build/%)
	@set -e; for t in $(TESTS); do echo "=== $$t ==="; ./build/$$t; done

build/%: test/%.cpp $(wildcard main/*.cpp main/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(wildcard main/putt_*.cpp) $< -o $@

replay: build/replay
build/replay: test/replay.cpp $(wildcard main/*.cpp main/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(wildcard main/putt_*.cpp) test/replay.cpp -o build/replay

clean:
	rm -rf build
```

- [ ] **Step 3: Verify the build skeleton runs**

Run: `mkdir -p main test && echo 'int main(){return 0;}' > test/test_vec3.cpp && make build/test_vec3 && ./build/test_vec3 && echo OK`
Expected: prints `OK` (this temporary file is overwritten in Task 2).

- [ ] **Step 4: Commit**

```bash
git add Makefile test/check.h
git commit -m "build: host test harness + Makefile"
```

---

## Task 2: Shared types + Vec3 math

**Files:**
- Create: `main/imu_types.h`
- Create: `main/vec3.h`
- Test: `test/test_vec3.cpp`

- [ ] **Step 1: Write the failing test**

Replace `test/test_vec3.cpp`:

```cpp
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `make build/test_vec3`
Expected: FAIL — `vec3.h` / `imu_types.h` not found.

- [ ] **Step 3: Write the types and math**

Create `main/imu_types.h` with the full **Data Contract** block above (all structs + `PuttDecision`, `DetectorConfig`, `PuttEvent`). Begin the file with:

```cpp
#pragma once
#include <cstdint>
```

Create `main/vec3.h` (ported from `main.ino:282-327`):

```cpp
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `make build/test_vec3 && ./build/test_vec3`
Expected: PASS — `N checks, 0 failures`.

- [ ] **Step 5: Commit**

```bash
git add main/imu_types.h main/vec3.h test/test_vec3.cpp
git commit -m "feat: shared IMU types + pure Vec3 math"
```

---

## Task 3: Preprocessor (raw counts → derived sample)

**Files:**
- Create: `main/putt_preprocess.h`, `main/putt_preprocess.cpp`
- Test: `test/test_preprocess.cpp`

- [ ] **Step 1: Write the failing test**

Create `test/test_preprocess.cpp`:

```cpp
#include "check.h"
#include "putt_preprocess.h"

// 1 g downward on Z at rest, no rotation. After enough samples gravity settles
// and linear accel → ~0, gyro → ~0.
static RawSample rest(uint32_t tUs) {
  // 1 g = 9.80665 m/s^2; counts = g_value / (0.000122 * 9.80665)
  const int16_t oneG = (int16_t)(1.0f / 0.000122f);  // ~8197 counts
  return RawSample{tUs, 0, 0, 0, 0, 0, oneG};
}

static void test_rest_settles() {
  Preprocessor pp{DetectorConfig{}};
  DerivedSample d{};
  for (int i = 0; i < 400; ++i) d = pp.process(rest(i * 5000));
  CHECK(d.valid);
  CHECK_NEAR(d.gyroDps, 0.0f, 0.5f);
  CHECK_NEAR(d.linearMag, 0.0f, 0.3f);  // gravity removed
}

static void test_gyro_scaling() {
  Preprocessor pp{DetectorConfig{}};
  // Feed rest first so bias settles to ~0, then a known rotation on X.
  for (int i = 0; i < 400; ++i) pp.process(rest(i * 5000));
  // 100 dps on X: counts = 100 / 0.0175
  const int16_t r = (int16_t)(100.0f / 0.0175f);
  DerivedSample d{};
  for (int i = 0; i < 50; ++i)  // let the low-pass converge
    d = pp.process(RawSample{(uint32_t)(2000000 + i*5000), r, 0, 0, 0, 0,
                             (int16_t)(1.0f/0.000122f)});
  CHECK_NEAR(d.gyroDps, 100.0f, 5.0f);
}

int main() {
  RUN(test_rest_settles);
  RUN(test_gyro_scaling);
  REPORT();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `make build/test_preprocess`
Expected: FAIL — `putt_preprocess.h` not found.

- [ ] **Step 3: Write the preprocessor**

Create `main/putt_preprocess.h`:

```cpp
#pragma once
#include "imu_types.h"

class Preprocessor {
public:
  explicit Preprocessor(const DetectorConfig &cfg) : cfg_(cfg) {}
  void reset();
  DerivedSample process(const RawSample &raw);

private:
  DetectorConfig cfg_;
  Vec3 gyroBias_{0, 0, 0};
  Vec3 filteredGyro_{0, 0, 0};
  Vec3 gravityAxis_{0, 0, 1};
  bool biasReady_ = false;
  bool gyroFilterReady_ = false;
  bool gravityReady_ = false;
};
```

Create `main/putt_preprocess.cpp` (ports `main.ino:330-352,1218-1248`):

```cpp
#include "putt_preprocess.h"
#include "vec3.h"

static const float GYRO_DPS_PER_LSB = 0.0175f;
static const float ACCEL_G_PER_LSB  = 0.000122f;
static const float DEG_TO_RAD       = 0.0174532925f;
static const float RAD_TO_DEG       = 57.2957795f;
static const float G                = 9.80665f;

void Preprocessor::reset() {
  gyroBias_ = {0, 0, 0};
  filteredGyro_ = {0, 0, 0};
  gravityAxis_ = {0, 0, 1};
  biasReady_ = gyroFilterReady_ = gravityReady_ = false;
}

DerivedSample Preprocessor::process(const RawSample &raw) {
  Vec3 gyroRaw{ raw.gx * GYRO_DPS_PER_LSB * DEG_TO_RAD,
                raw.gy * GYRO_DPS_PER_LSB * DEG_TO_RAD,
                raw.gz * GYRO_DPS_PER_LSB * DEG_TO_RAD };
  Vec3 accel{ raw.ax * ACCEL_G_PER_LSB * G,
              raw.ay * ACCEL_G_PER_LSB * G,
              raw.az * ACCEL_G_PER_LSB * G };

  // Gyro bias (slow EMA toward the resting value).
  if (!biasReady_) { gyroBias_ = gyroRaw; biasReady_ = true; }
  else gyroBias_ = add3(scale3(gyroBias_, 1.0f - cfg_.gyroBiasAlpha),
                        scale3(gyroRaw, cfg_.gyroBiasAlpha));

  // Bias-corrected + low-pass filtered gyro.
  Vec3 corrected = sub3(gyroRaw, gyroBias_);
  if (!gyroFilterReady_) { filteredGyro_ = corrected; gyroFilterReady_ = true; }
  else filteredGyro_ = add3(scale3(filteredGyro_, 1.0f - cfg_.gyroFilterAlpha),
                            scale3(corrected, cfg_.gyroFilterAlpha));

  // Gravity estimate (only when |accel| looks like ~1 g).
  float accelMag = mag3(accel);
  if (accelMag >= cfg_.gravityMinMps2 && accelMag <= cfg_.gravityMaxMps2) {
    Vec3 axis = normalize3(accel);
    if (!gravityReady_) { gravityAxis_ = axis; gravityReady_ = true; }
    else gravityAxis_ = normalize3(add3(scale3(gravityAxis_, 1.0f - cfg_.gravityAlpha),
                                        scale3(axis, cfg_.gravityAlpha)));
  }

  Vec3 linear = gravityReady_
      ? sub3(accel, scale3(gravityAxis_, dot3(accel, gravityAxis_)))
      : Vec3{0, 0, 0};

  DerivedSample d;
  d.tUs = raw.tUs;
  d.gyroRad = filteredGyro_;
  d.gyroDps = mag3(filteredGyro_) * RAD_TO_DEG;
  d.linearMps2 = linear;
  d.linearMag = mag3(linear);
  d.valid = biasReady_ && gravityReady_;
  return d;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `make build/test_preprocess && ./build/test_preprocess`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add main/putt_preprocess.h main/putt_preprocess.cpp test/test_preprocess.cpp
git commit -m "feat: pure IMU preprocessing pipeline"
```

---

## Task 4: Feature extraction

**Files:**
- Create: `main/putt_features.h`, `main/putt_features.cpp`
- Test: `test/test_features.cpp`

`extractFeatures` operates over a contiguous array of `DerivedSample` (the candidate window plus a margin of quiet samples on each side, supplied by the detector in Task 5). It is pure and deterministic.

- [ ] **Step 1: Write the failing test (synthetic putt vs. synthetic walk)**

Create `test/test_features.cpp`:

```cpp
#include "check.h"
#include "putt_features.h"
#include <vector>
#include <cmath>

static const float DEG_TO_RAD = 0.0174532925f;

// Build a single-axis pendulum: back (negative) then forward (positive) lobe on X,
// gravity already removed (linear ~0). dt = 5 ms.
static std::vector<DerivedSample> makePutt() {
  std::vector<DerivedSample> v;
  auto push = [&](uint32_t tUs, float dpsX, float lin) {
    Vec3 g{dpsX * DEG_TO_RAD, 0, 0};
    v.push_back(DerivedSample{tUs, g, std::fabs(dpsX), {lin, 0, 0}, std::fabs(lin), true});
  };
  uint32_t t = 0;
  for (int i = 0; i < 40; ++i) { push(t, 0, 0); t += 5000; }          // pre-still
  for (int i = 0; i < 30; ++i) { push(t, -20*std::sin(M_PI*i/30), 0); t += 5000; } // backswing
  for (int i = 0; i < 30; ++i) { push(t, 60*std::sin(M_PI*i/30), 0); t += 5000; }  // forward (bell)
  for (int i = 0; i < 40; ++i) { push(t, 0, 0); t += 5000; }          // post-settle
  return v;
}

// Walking: multi-axis, many reversals, never settles.
static std::vector<DerivedSample> makeWalk() {
  std::vector<DerivedSample> v;
  uint32_t t = 0;
  for (int i = 0; i < 140; ++i) {
    float a = 30*std::sin(2*M_PI*i/12);
    float b = 25*std::sin(2*M_PI*i/9 + 1.0);
    Vec3 g{a*DEG_TO_RAD, b*DEG_TO_RAD, 0};
    float dps = std::sqrt(a*a + b*b);
    v.push_back(DerivedSample{t, g, dps, {2.0f, 1.5f, 0}, 2.5f, true});
    t += 5000;
  }
  return v;
}

static void test_putt_features() {
  auto p = makePutt();
  PuttFeatures f = extractFeatures(p.data(), (int)p.size());
  CHECK(f.axisConsistency > 0.9f);     // single axis
  CHECK(f.reversalCount <= 2);         // one clean back->forward
  CHECK(f.peakForwardDps > 40.0f);
  CHECK(f.bellRatio > 1.4f);
  CHECK(f.linAngRatio < 0.3f);         // mostly rotation
  CHECK(f.preStillMs > 100.0f);
  CHECK(f.postSettleMs > 100.0f);
}

static void test_walk_features() {
  auto w = makeWalk();
  PuttFeatures f = extractFeatures(w.data(), (int)w.size());
  // Walk must fail at least one strong discriminator:
  CHECK(f.axisConsistency < 0.85f || f.reversalCount > 3 || f.linAngRatio > 0.6f);
}

int main() {
  RUN(test_putt_features);
  RUN(test_walk_features);
  REPORT();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `make build/test_features`
Expected: FAIL — `putt_features.h` not found.

- [ ] **Step 3: Write the feature extractor**

Create `main/putt_features.h`:

```cpp
#pragma once
#include "imu_types.h"

// samples: contiguous window including quiet margins on both ends.
PuttFeatures extractFeatures(const DerivedSample *samples, int n);
```

Create `main/putt_features.cpp`:

```cpp
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
  f.impactPresent = false;  // decided against config threshold in Task 5

  return f;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `make build/test_features && ./build/test_features`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add main/putt_features.h main/putt_features.cpp test/test_features.cpp
git commit -m "feat: window feature extraction (axis, reversals, bell, lin/ang, impact)"
```

---

## Task 5: Detector (buffer + candidate FSM + decision)

**Files:**
- Create: `main/putt_detector.h`, `main/putt_detector.cpp`
- Test: `test/test_detector.cpp`

- [ ] **Step 1: Write the failing test**

Create `test/test_detector.cpp`:

```cpp
#include "check.h"
#include "putt_detector.h"
#include <cmath>

static const float DEG2R = 0.0174532925f;
static const int16_t ONE_G = (int16_t)(1.0f / 0.000122f);

// Feed a synthetic single-axis pendulum (with a small impact bump) as RAW samples.
static int feedPutt(PuttDetector &det) {
  int detections = 0;
  uint32_t t = 0;
  auto step = [&](float dpsX) {
    int16_t gx = (int16_t)(dpsX / 0.0175f);
    PuttEvent e = det.update(RawSample{t, gx, 0, 0, 0, 0, ONE_G});
    if (e.detected && e.decision == PuttDecision::Accept) ++detections;
    t += 5000;
  };
  for (int i = 0; i < 200; ++i) step(0);                     // settle filters + pre-still
  for (int i = 0; i < 30; ++i) step(-25*std::sin(M_PI*i/30));// backswing
  for (int i = 0; i < 30; ++i) step(70*std::sin(M_PI*i/30)); // forward
  for (int i = 0; i < 80; ++i) step(0);                      // settle (closes candidate)
  return detections;
}

// Walking never settles → many reversals / multi-axis → must NOT detect.
static int feedWalk(PuttDetector &det) {
  int detections = 0;
  uint32_t t = 0;
  for (int i = 0; i < 200; ++i) {
    float a = 35*std::sin(2*M_PI*i/12), b = 30*std::sin(2*M_PI*i/9 + 1.0);
    int16_t gx = (int16_t)(a / 0.0175f), gy = (int16_t)(b / 0.0175f);
    PuttEvent e = det.update(RawSample{t, gx, gy, 0, 1500, 1200, ONE_G});
    if (e.detected && e.decision == PuttDecision::Accept) ++detections;
    t += 5000;
  }
  return detections;
}

static void test_detects_putt() {
  PuttDetector det{DetectorConfig{}};
  CHECK(feedPutt(det) == 1);
}

static void test_rejects_walk() {
  PuttDetector det{DetectorConfig{}};
  CHECK(feedWalk(det) == 0);
}

int main() {
  RUN(test_detects_putt);
  RUN(test_rejects_walk);
  REPORT();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `make build/test_detector`
Expected: FAIL — `putt_detector.h` not found.

- [ ] **Step 3: Write the detector**

Create `main/putt_detector.h`:

```cpp
#pragma once
#include "imu_types.h"
#include "putt_preprocess.h"

class PuttDetector {
public:
  explicit PuttDetector(const DetectorConfig &cfg)
    : cfg_(cfg), pre_(cfg) {}
  void reset();
  // Feed one raw sample; returns an event with detected=true only on the
  // sample that closes (and decides) a candidate window.
  PuttEvent update(const RawSample &raw);

private:
  static const int CAP = 1024;            // ~5 s at 200 Hz
  PuttEvent decide();                     // run features+gates on current window

  DetectorConfig cfg_;
  Preprocessor pre_;
  DerivedSample buf_[CAP];
  int count_ = 0;                         // valid samples held (capped)
  bool inCandidate_ = false;
  int candStart_ = 0;                     // index in buf_ where motion opened
  uint32_t lastQuietStartUs_ = 0;
  bool quietRunning_ = false;
  uint32_t lastTUs_ = 0;
  bool haveLast_ = false;
};
```

Create `main/putt_detector.cpp`:

```cpp
#include "putt_detector.h"
#include "putt_features.h"
#include <cmath>

void PuttDetector::reset() {
  pre_.reset();
  count_ = 0; inCandidate_ = false; candStart_ = 0;
  quietRunning_ = false; haveLast_ = false;
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

  if (!inCandidate_) {
    if (d.gyroDps >= cfg_.candidateRiseDps) {
      inCandidate_ = true;
      // Start a few samples back to capture pre-still + ramp.
      candStart_ = count_ - 1;
      int back = 0;
      while (candStart_ > 0 && back < 40) { --candStart_; ++back; }
      quietRunning_ = false;
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
    ev.decision = PuttDecision::Reject; ev.reason = why; return ev;
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
  return ev;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `make build/test_detector && ./build/test_detector`
Expected: PASS — putt detected exactly once, walk rejected.

- [ ] **Step 5: Run the whole suite + commit**

Run: `make test`
Expected: all four test binaries report `0 failures`.

```bash
git add main/putt_detector.h main/putt_detector.cpp test/test_detector.cpp
git commit -m "feat: PuttDetector buffer + candidate FSM + gated decision"
```

---

## Task 6: Replay harness

**Files:**
- Create: `test/replay.cpp`

- [ ] **Step 1: Write the replay tool**

Create `test/replay.cpp`. CSV format: `t_us,gx,gy,gz,ax,ay,az` (one row per sample; `#` comment lines skipped).

```cpp
#include "putt_detector.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: replay <clip.csv>\n"); return 2; }
  FILE *fp = std::fopen(argv[1], "r");
  if (!fp) { std::perror("open"); return 2; }

  PuttDetector det{DetectorConfig{}};
  char line[256];
  int detections = 0, rejects = 0, rows = 0;
  while (std::fgets(line, sizeof(line), fp)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    RawSample r{};
    int gx, gy, gz, ax, ay, az; unsigned long t;
    if (std::sscanf(line, "%lu,%d,%d,%d,%d,%d,%d",
                    &t, &gx, &gy, &gz, &ax, &ay, &az) != 7) continue;
    r.tUs = (uint32_t)t;
    r.gx=gx; r.gy=gy; r.gz=gz; r.ax=ax; r.ay=ay; r.az=az;
    ++rows;
    PuttEvent e = det.update(r);
    if (!e.detected) continue;
    if (e.decision == PuttDecision::Accept) {
      ++detections;
      std::printf("ACCEPT t=%u dur=%ums axis=%.2f rev=%d fwd=%.1f bell=%.2f "
                  "lin/ang=%.2f jerk=%.2f\n",
                  r.tUs, e.features.durationMs, e.features.axisConsistency,
                  e.features.reversalCount, e.features.peakForwardDps,
                  e.features.bellRatio, e.features.linAngRatio, e.features.impactJerk);
    } else {
      ++rejects;
      std::printf("REJECT(%s) t=%u dur=%ums axis=%.2f rev=%d lin/ang=%.2f\n",
                  e.reason, r.tUs, e.features.durationMs, e.features.axisConsistency,
                  e.features.reversalCount, e.features.linAngRatio);
    }
  }
  std::fclose(fp);
  std::printf("# %s: %d rows, %d accepted, %d rejected\n", argv[1], rows, detections, rejects);
  return 0;
}
```

- [ ] **Step 2: Build and smoke-test**

Run:
```bash
make replay
printf '# smoke\n0,0,0,0,0,0,8197\n5000,0,0,0,0,0,8197\n' > /tmp/smoke.csv
./build/replay /tmp/smoke.csv
```
Expected: `# /tmp/smoke.csv: 2 rows, 0 accepted, 0 rejected` (too short to settle; no crash).

- [ ] **Step 3: Commit**

```bash
git add test/replay.cpp
git commit -m "feat: host replay harness for recorded clips"
```

---

## Task 7: Firmware — raw CSV logging for data capture

**Files:**
- Modify: `main/main.ino` (logging section)

This produces the corpus. It is additive — it does not change existing detection yet.

- [ ] **Step 1: Add a raw-log emitter gated by the existing flag**

In `main/main.ino`, find `ENABLE_RAW_STROKE_LOG` (`main.ino:30`) and set it to `true`. In the main IMU read path (where `readImu(sample)` succeeds, around `main.ino:1203` caller), add — using the *raw register values* before scaling. Refactor `readImu` to also expose raw counts, or add a sibling `logRawSample`:

```cpp
// Called once per successful IMU read, with raw register counts.
static void logRawSample(uint32_t tUs, int16_t gx, int16_t gy, int16_t gz,
                         int16_t ax, int16_t ay, int16_t az) {
  if (!ENABLE_RAW_STROKE_LOG) return;
  Serial.print(tUs);   Serial.print(',');
  Serial.print(gx);    Serial.print(',');
  Serial.print(gy);    Serial.print(',');
  Serial.print(gz);    Serial.print(',');
  Serial.print(ax);    Serial.print(',');
  Serial.print(ay);    Serial.print(',');
  Serial.println(az);
}
```

In `readImu` (`main.ino:1209-1223`), capture the raw `int16_t` values into out-params or a struct member so the caller can pass them to `logRawSample` with a `micros()` timestamp.

- [ ] **Step 2: Verify it compiles for the board**

Run: `arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense main`
Expected: compiles with no errors. (If `arduino-cli` is unavailable, compile via the Arduino IDE and confirm a clean build.)

- [ ] **Step 3: Capture the corpus**

Flash, open serial @ 115200, and record one CSV file per action into `test/fixtures/`. Capture procedure for each clip: start logging, perform the action once with ~1 s of stillness before and after, stop.

Positives (≥5 each): `putt_short_NN.csv`, `putt_long_NN.csv`, `putt_fast_NN.csv`, `putt_offcenter_NN.csv`.
Negatives (≥5 each): `neg_pickup_NN.csv`, `neg_setdown_NN.csv`, `neg_walk_NN.csv`, `neg_waggle_NN.csv`, `neg_taptap_NN.csv`.

Strip any non-CSV serial lines so each file is pure `t_us,gx,...,az`.

- [ ] **Step 4: Commit the corpus**

```bash
git add main/main.ino test/fixtures/*.csv
git commit -m "feat: raw IMU CSV logging + recorded clip corpus"
```

---

## Task 8: Corpus regression test + threshold tuning

**Files:**
- Create: `test/run_corpus.sh`

- [ ] **Step 1: Write the corpus runner**

Create `test/run_corpus.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
make replay >/dev/null
fail=0
for f in test/fixtures/putt_*.csv; do
  if ! ./build/replay "$f" | grep -q '^ACCEPT'; then
    echo "MISS  (expected detect) $f"; fail=1
  fi
done
for f in test/fixtures/neg_*.csv; do
  if ./build/replay "$f" | grep -q '^ACCEPT'; then
    echo "FALSE (expected reject) $f"; fail=1
  fi
done
[ "$fail" = 0 ] && echo "CORPUS PASS: all positives detected, no false positives"
exit $fail
```

- [ ] **Step 2: Run it and read the failures**

Run: `bash test/run_corpus.sh`
Expected (first run): a list of `MISS`/`FALSE` lines — the starting error set.

- [ ] **Step 3: Tune thresholds in `DetectorConfig`**

Use `./build/replay <clip>` on individual misses/false-positives to read the feature values, then adjust the gate constants in `DetectorConfig` (`main/imu_types.h`) so positives sit on the accept side and negatives on the reject side with margin. Re-run `bash test/run_corpus.sh` after each change. Decide `requireImpact`: inspect `jerk=` on positive vs. negative clips — if every positive's jerk clearly exceeds every negative's, set `requireImpact = true` and `impactJerkMps3` between the two populations; otherwise leave it `false`.

- [ ] **Step 4: Confirm green + commit**

Run: `bash test/run_corpus.sh && make test`
Expected: `CORPUS PASS` and all unit tests `0 failures`.

```bash
git add test/run_corpus.sh main/imu_types.h
git commit -m "test: corpus regression gate + tuned detection thresholds"
```

---

## Task 9: Firmware — wire PuttDetector into main.ino

**Files:**
- Modify: `main/main.ino`

- [ ] **Step 1: Replace the detection glue**

In `main/main.ino`: `#include "putt_detector.h"`. Add a file-scope `static PuttDetector g_detector{DetectorConfig{}};`. In the sample loop, after reading raw counts, build a `RawSample` (with `micros()` timestamp + raw `int16_t` counts) and call:

```cpp
PuttEvent ev = g_detector.update(raw);
if (ev.detected && ev.decision == PuttDecision::Accept) {
  Serial.println("PUTT_DETECTED");
  // (existing result-page / display hooks fed from ev.features)
} else if (ev.detected) {
  Serial.print("PUTT_REJECTED,reason="); Serial.println(ev.reason);
}
```

Remove the old `SwingStats`/state-machine detection path (`finishSwing`, `updateReady`, `updateSwing`, candidate-buffer analysis) now that `PuttDetector` owns detection. Keep IMU init, display, and serial scaffolding. The old `gyroBias`/`gravityAxis`/`filteredGyro` globals and their updaters (`main.ino:330-352,1227-1248`) are now dead — delete them; `Preprocessor` owns that state.

- [ ] **Step 2: Compile for the board**

Run: `arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense main`
Expected: clean build.

- [ ] **Step 3: On-device sanity check**

Flash. Verify: real putts print `PUTT_DETECTED`; pickup/walk/waggle print `PUTT_REJECTED,reason=...` (or nothing). Spot-check that the reasons match the corpus expectations.

- [ ] **Step 4: Commit**

```bash
git add main/main.ino
git commit -m "feat: drive firmware detection from pure PuttDetector"
```

---

## Notes for the executor

- **TDD order matters:** every `putt_*` module is proven on host before it touches the board. Do not skip the red→green→commit cycle.
- **No STL in `main/` modules** (fixed arrays only) so they stay flashable to the nRF52840.
- **The corpus is the source of truth.** If a threshold change makes the unit tests and corpus both green, it is correct by definition. Never tune by feel on live hardware.
- **High-ODR impact measurement (optional, only if impact is unreliable):** temporarily reconfigure the accel ODR in `configureImu` to a high rate and capture a few putt clips to inspect the impact transient's shape before deciding whether 200 Hz suffices for `requireImpact`. This informs Task 8 step 3; it is a measurement, not a code change to keep.
- **Deferred:** face-angle / tempo / path metric accuracy is out of scope (see spec). `ev.features` already carries enough to revisit metrics later.
- **Simplification vs. spec:** the spec described "hard gates + a margin score." This plan implements **gates only**, each with a named reject reason — more interpretable and directly provable against the corpus. If, during Task 8 tuning, a single threshold can't cleanly separate positives from negatives (genuine borderline overlap), reintroduce a weighted margin score over the same features as a tie-breaker. Don't add it pre-emptively.
