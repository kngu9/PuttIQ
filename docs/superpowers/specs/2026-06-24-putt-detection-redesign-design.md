# Putt Detection Redesign — Design

**Date:** 2026-06-24
**Status:** Approved (design), pending implementation plan
**Platform:** Seeed XIAO nRF52840 Sense, LSM6DS3/LSM6DS3C IMU, Arduino C++

## Problem

The current detector (in a single ~2,600-line `main/main.ino`) suffers from:

1. **False positives** — it fires on non-putt actions, specifically **picking up / setting
   down the putter, walking with it in hand, and waggling/fidgeting at address**. It does
   *not* struggle with practice strokes (those are out of scope here).
2. **Inaccurate metrics** (face angle, tempo, path) — acknowledged but **deferred**; there
   is no point trusting metrics on a stroke that may not be a real putt.
3. **Fiddly arming** — the auto-arm-after-stillness sequence accidentally arms during pickup
   and settling.

The device is mounted on the **butt end of the putter grip**, so a real stroke is a
pendulum rotation about a fairly consistent axis. The three false-positive actions are all
**mechanically distinct from a real putt and none of them strike a ball**, giving us two
independent discriminators (kinematic shape and ball impact). Winning on either kills the
false positives.

## Goal

Fully automatic detection (no user arming) that emits a putt event **only** for real
strokes. Manual address-trigger is the fallback if automatic proves infeasible. Metrics
accuracy is explicitly out of scope for this work.

## Approach

Approach B: an **impact-gated envelope detector** built behind an **offline replay
harness** so detection can be tuned and proven against recorded data instead of guessed at
on live hardware.

## Architecture

Split detection out of `main.ino` into a **pure, host-testable module**.

```
main/
  main.ino              # thin: IMU read loop + serial/output glue only
  putt_detector.h/.cpp  # PURE detection module — no Arduino/Wire deps
  imu_sample.h          # plain data types (raw sample, detection result)
test/
  replay.cpp            # host tool: reads a recorded log, feeds the module, prints events
  test_features.cpp     # host unit tests for feature extraction + decision
  fixtures/             # recorded labeled clips (putts + pickup/walk/waggle)
```

**Module boundary:**

- **Input:** raw IMU samples only — `{ timestamp_us, gyro[3], accel[3] }` (raw integer
  counts, pre-scaling). The module does its *own* gyro-bias estimation, gravity estimation,
  and linear-accel computation (the filters currently in `main.ino`, moved in).
- **Output:** events — `PuttDetected{features...}` or nothing.
- **Why raw in, not pre-filtered:** the replay harness feeds the module exactly what the
  hardware saw, so a recorded clip reproduces the live result with no hidden state in
  `main.ino`.

`main.ino` shrinks to: init IMU → read sample → `detector.update(sample)` → on event, emit
serial/output.

## Data capture & replay harness (foundation, built first)

**Capture format.** Enable raw logging (the existing `ENABLE_RAW_STROKE_LOG` flag). One CSV
line per sample over serial @ 115200:

```
t_us, gx, gy, gz, ax, ay, az
```

Raw integer counts so replay reproduces the exact pipeline. 200 Hz × 7 fields fits in
115200 baud.

**Capture-rate measurement (data-collection phase only).** The ball impact is a brief
transient that 200 Hz (5 ms/sample) may undersample. Record a few clips with the
accelerometer at high ODR (LSM6DS3 supports up to 6.6 kHz) to characterize the impact's
frequency content and decide whether 200 Hz suffices for the impact gate, or whether
production needs the IMU FIFO / higher accel rate. This is a measurement, not a commitment.

**Labeled clip library** in `test/fixtures/`, named by label:

- **Positives:** real putts — short/long, fast/slow, off-center, left/right.
- **Negatives:** pickup, set-down, walking with putter in hand, waggles/fidgeting at
  address, tapping the ball back.

**Replay harness (`replay.cpp`).** Host program compiled with plain `g++` (no Arduino) that
reads a clip, feeds samples through the real `putt_detector` module, and prints the events
it would emit.

**Regression target:** **100% of positive clips detected, 0 negatives triggered.** Every
tuning change re-runs against the whole corpus.

## Detection algorithm — impact-gated envelope detector

The detector runs free (no arming). A ring buffer holds the last few seconds of raw samples
plus derived signals. Detection is **retroactive**: wait for a candidate stroke to finish,
then look back over the buffered window and decide.

```
free-running buffer
   → detect "something happened and then settled" (candidate window)
   → extract features over that window
   → apply gates + score
   → emit PuttDetected only if it passes
```

**Candidate window.** A region where motion rose above a low floor, peaked, and settled
back to quiet on both ends. Deliberately loose — it bounds *what* to analyze, it does not
decide.

**Features** (each independently testable and physically meaningful):

1. **Axis consistency** — fraction of angular-velocity energy along the dominant rotation
   axis. Putt ≈ single axis (high); walking = multi-axis (low). *Kills walking.*
2. **Reversal count** about the dominant axis — putt = one clean back→forward reversal
   (≈1–2 with follow-through); waggle = many. *Kills waggling.*
3. **Forward-stroke profile** — one smooth bell-shaped angular-velocity peak, not lumpy or
   monotonic.
4. **Linear-to-angular ratio** — putt is mostly rotation with little net translation;
   pickup/set-down = large sustained linear motion. *Kills pickup/set-down.*
5. **Pre-stillness & post-settle** — quiet address before, decay to quiet after; walking
   never settles.
6. **Bounds** — duration, peak velocity, amplitude within plausible putt ranges.
7. **Impact transient** — a jerk spike near the forward peak; the hard gate none of the
   false cases produce.

**Decision = hard gates + margin score.** Gates (must pass): impact present (subject to the
data-decides caveat below), axis consistency above floor, linear/angular ratio below
ceiling, bounds satisfied. Remaining features combine into a margin score for borderline
cases. **Every threshold is derived from the recorded corpus**, set to the values that
separate positives from negatives, with the replay harness proving the separation.

**Impact gate — "data decides."** Impact is a hard requirement **if** the recorded data
shows 200 Hz can detect it reliably. If gentle putts are undersampled, we have two outs
before falling back to manual arming: (a) raise accel ODR / use the IMU FIFO to capture
impact at high rate while detecting at 200 Hz, or (b) demote impact to one weighted signal
and let envelope features #1–#6 (which independently separate all three false-positive
types) carry the load. Manual address-trigger is the last resort.

**Latency note.** Retroactive analysis means `PuttDetected` fires a fraction of a second
after impact. Acceptable for logging/feedback (not live-at-impact haptics).

## Edge cases & error handling

- **Steep / tilted hold** — the gravity estimate only updates when accel magnitude is
  7–12.5 m/s² and otherwise fails silently (known issue). Track gravity-estimate validity;
  when unreliable for a window, fall back to gyro-only features (axis + reversals work
  without a good gravity vector).
- **Back-to-back putts** — a short result-hold/refractory period so one stroke can't emit
  twice and a quick second putt isn't swallowed.
- **Timing jitter / dropped samples** — features use actual timestamps, not assumed 5 ms
  spacing; a gap larger than a threshold invalidates the in-progress candidate.
- **No IMU / startup** — module reports "not ready" until it has enough samples and a
  settled gravity estimate.

## Testing strategy (host-side, no hardware in the loop)

- **Unit tests** (`test_features.cpp`) — each feature extractor against synthetic signals
  with known answers (pure single-axis sine → axis consistency ≈ 1.0; two-tone wobble →
  high reversal count).
- **Replay/corpus tests** — every clip in `fixtures/` run through `replay.cpp`; assert
  detect on positives, no-detect on negatives. Regression gate: 100% positives, 0
  negatives.
- **Thresholds are a committed test artifact** — they live alongside the corpus; any change
  that breaks separation shows up immediately.

## Out of scope (deferred)

- Face-angle / tempo / path **metric accuracy**. Fix detection first; metrics get their own
  pass once a detected stroke is trustworthy.
- Practice-stroke discrimination (not a reported problem).
- On-device ML classifier (Approach C) — the recorded corpus is its prerequisite; revisit
  only if the gated/envelope detector proves insufficient.
