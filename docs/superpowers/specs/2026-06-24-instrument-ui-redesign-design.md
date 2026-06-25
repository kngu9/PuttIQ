# Instrument UI Redesign — Design

**Date:** 2026-06-24
**Status:** Approved (design), pending implementation plan
**Platform:** Seeed XIAO nRF52840 Sense, 240×240 round TFT (Seeed round display), Arduino C++

## Problem

The current UI stacks text lines on a rectangular grid (y = 72, 106, 138 …) on a
*round* display, so content crowds the corners that the circle clips. There are three
somewhat-redundant result pages (Tempo / Trace / Stats), the `[ZERO]` calibration target
is a tiny tap zone, and every screen is drawn twice (a `screenSprite` path and a duplicate
direct-`tft` path), making the display code in `main.ino` large and error-prone.

## Goal

A holistic redesign of the visual style, result presentation, and touch interaction for the
round display. **Out of scope:** live during-stroke feedback (explicitly excluded by the
user). The detection algorithm is unchanged; this is the UI around it.

## Visual language (applies to every screen)

- Black background; a single thin white ring hugging the bezel as the frame.
- One accent color: **amber**, used *only* for the hero value and the impact point.
  Everything else is white/grey. The restraint is what makes it read as premium ("instrument"
  aesthetic — a refined watch-face look).
- All content stays within a safe inner circle (≈ Ø170, i.e. radius ~85 from center 120,120)
  so nothing clips on the round edge. This fixes today's corner-crowding.
- **Typography constraint (honest):** the embedded font is bitmap (TFT size 1/2/3), so the
  "thin/crisp" feel comes from small labels + large values + generous radial spacing + a
  tight palette — not a true thin typeface.

## Flow & navigation

- States: **Idle/Ready → (putt detected) → Result**. Detection is automatic (ready = armed).
- The Result is **two swipeable pages** (swipe left/right). Page dots at the bottom show
  which page is active.
- A large **Exit pill** (bottom) dismisses the result back to Idle.
- On detection, a brief **"PUTT" confirmation** (~0.5 s, amber) shows, then the trace page
  appears. This is the result moment, not live-during-stroke feedback.

## Page 1 — Trace (hero)

- The clubhead arc (from `OrientationTracker.headPoint`, already implemented) drawn large
  and centered in white (~2–3 px), spanning most of the face.
- A faint grey **target reference line** straight through the center, so the arc's deviation
  from straight is visible at a glance.
- An **amber filled dot** marks the impact point on the arc.
- Top chip: tiny `FACE` label + large amber value (e.g. `1.2°R`). Bottom chip: `2.1:1` with a
  tiny `TEMPO` label below.
- Two page dots at the very bottom (page 1 active).

## Page 2 — Details

- Tiny `DETAILS` header at top.
- Clean stacked rows within the safe circle, label left / value right:
  `FACE 1.2°R`, `PATH 0.5° OUT`, `TEMPO 2.1:1`, `B/F 480/230`, `DUR 710ms`, `IMPACT +180`.
  (Face/path use the calibration offsets `faceZeroDeg`/`pathZeroDeg`.)
- A proper finger-sized **`[ZERO]` pill** (amber outline) for calibration — much larger
  touch target than today.
- Exit pill + page dots at the bottom.

## Idle / error states

- **Ready:** minimal — `PuttIQ` wordmark + a small amber "ready" dot. Calm, not busy.
- **No IMU:** centered warning in amber.

## Architecture / code structure

Extract the display rendering out of `main.ino` into a small **`ui_render`** module with a
single draw path (sprite only — drop the duplicated direct-`tft` path). The module exposes
intent-level entry points, e.g.:

- `ui.showReady()`
- `ui.showDetected()` (the brief PUTT confirmation)
- `ui.showTracePage(const UiResult&)`
- `ui.showDetailsPage(const UiResult&)`
- `ui.showNoImu()`

where `UiResult` is a small plain struct carrying the values a result screen needs (trace
points + count, face/path/tempo/durations/impact, impact trace index). This isolates layout
from detection/firmware glue, removes the sprite-vs-tft duplication, and shrinks `main.ino`.

Shared geometry constants (center 120,120; safe radius; ring radius; palette: BLACK, WHITE,
GREY, AMBER) live in the module.

**Constraint:** `ui_render` may use Arduino/TFT APIs (it is firmware-side, not the pure
host-tested detection modules). It does not need host unit tests; correctness is verified
on-device. Pure geometry helpers (e.g. mapping a trace point into screen space, fitting the
arc within the safe circle) should be small and self-evidently correct.

## Out of scope (deferred)

- Live during-stroke feedback.
- Persisting calibration across power cycles (the `[ZERO]` offset remains RAM-only for now).
- Changes to the detection algorithm or metrics math.
