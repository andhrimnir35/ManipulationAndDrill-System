#include "breakpoint_math.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool nearValue(float actual, float expected, float tolerance = 0.001f) {
  return fabsf(actual - expected) <= tolerance;
}

int main() {
  BrakePointApproach approach = BrakePointApproach::Inactive;

  // Acceptance: a fast 117 -> 123 crossing keeps the original lower side.
  assert(bpSegmentCrossed(117.0f, 123.0f, 120.0f, approach));
  assert(approach == BrakePointApproach::FromBelow);
  assert(bpDirectionForbidden(approach, +1));
  assert(!bpDirectionForbidden(approach, -1));

  // Even after overshooting, only motion back toward the original side is free.
  float originalSideDistance = 0.0f;
  assert(!bpOriginalSideDistance(approach, 120.0f, 123.0f,
                                 originalSideDistance));
  assert(bpOriginalSideDistance(approach, 120.0f, 119.0f,
                                originalSideDistance));
  assert(nearValue(originalSideDistance, 1.0f));

  // Acceptance: crossing from above blocks decreasing angle and permits escape.
  approach = BrakePointApproach::Inactive;
  assert(bpSegmentCrossed(123.0f, 117.0f, 120.0f, approach));
  assert(approach == BrakePointApproach::FromAbove);
  assert(bpDirectionForbidden(approach, -1));
  assert(!bpDirectionForbidden(approach, +1));

  // 0/360 crossings use circular segment math.
  approach = BrakePointApproach::Inactive;
  assert(bpSegmentCrossed(359.0f, 1.0f, 0.0f, approach));
  assert(approach == BrakePointApproach::FromBelow);

  // Predictive lead: reaction + v^2/(2a) + per-axis margin.
  const float j1Lead = bpPredictedLeadDistanceDeg(100.0f, 40, 500, 0.6f,
                                                  1.0f, 45.0f);
  const float j2Lead = bpPredictedLeadDistanceDeg(100.0f, 40, 350, 0.8f,
                                                  1.0f, 45.0f);
  const float j3Lead = bpPredictedLeadDistanceDeg(100.0f, 40, 650, 0.6f,
                                                  1.0f, 45.0f);
  assert(nearValue(j1Lead, 14.6f));
  assert(j2Lead > j1Lead);
  assert(j1Lead > j3Lead);
  assert(nearValue(bpPredictedLeadDistanceDeg(0.0f, 40, 500, 0.6f,
                                              1.0f, 45.0f),
                   1.0f));
  assert(nearValue(bpPredictedLeadDistanceDeg(1000.0f, 40, 50, 20.0f,
                                              1.0f, 45.0f),
                   45.0f));

  // Directed gates remain deterministic through the 0/360 wrap.
  approach = BrakePointApproach::Inactive;
  assert(bpSegmentCrossedForMode(BrakePointMode::BlockIncrease,
                                 359.0f, 1.0f, 0.0f, approach));
  assert(approach == BrakePointApproach::FromBelow);
  approach = BrakePointApproach::Inactive;
  assert(!bpSegmentCrossedForMode(BrakePointMode::BlockDecrease,
                                  359.0f, 1.0f, 0.0f, approach));
  assert(bpPathCrossesForbidden(BrakePointMode::BlockDecrease,
                                1.0f, 359.0f, 0.0f));
  assert(!bpPathCrossesForbidden(BrakePointMode::BlockIncrease,
                                 1.0f, 359.0f, 0.0f));

  // Circular directional distances remain correct through the 0/360 wrap.
  assert(nearValue(bpDirectionalDistanceDeg(359.0f, 1.0f, +1), 2.0f));
  assert(nearValue(bpDirectionalDistanceDeg(1.0f, 359.0f, -1), 2.0f));
  assert(!bpPathCrossesForbidden(BrakePointMode::BlockIncrease,
                                 350.0f, 10.0f, 60.0f));
  assert(bpPathCrossesForbidden(BrakePointMode::BlockIncrease,
                                350.0f, 100.0f, 60.0f));
  assert(bpPathCrossesForbidden(BrakePointMode::BlockDecrease,
                                10.0f, 250.0f, 300.0f));
  assert(bpPathCrossesForbidden(BrakePointMode::Wall,
                                80.0f, 100.0f, 90.0f));
  assert(bpPathCrossesForbidden(BrakePointMode::Wall,
                                100.0f, 80.0f, 90.0f));

  // Command preflight and live sample crossing must agree for every normal
  // (<180 degree) circular movement, including wrap-around.
  const BrakePointMode modes[] = {
      BrakePointMode::Wall,
      BrakePointMode::BlockIncrease,
      BrakePointMode::BlockDecrease};
  for (BrakePointMode mode : modes) {
    for (int start = 0; start < 360; start += 7) {
      for (int delta = -175; delta <= 175; delta += 13) {
        if (delta == 0) {
          continue;
        }
        const float target = bpNormalize360(
            static_cast<float>(start + delta));
        for (int boundary = 0; boundary < 360; boundary += 11) {
          BrakePointApproach sampledApproach =
              BrakePointApproach::Inactive;
          const bool sampled = bpSegmentCrossedForMode(
              mode,
              static_cast<float>(start),
              target,
              static_cast<float>(boundary),
              sampledApproach);
          const bool preflight = bpPathCrossesForbidden(
              mode,
              static_cast<float>(start),
              target,
              static_cast<float>(boundary));
          assert(sampled == preflight);
        }
      }
    }
  }

  puts("breakpoint safety host tests: PASS");
  return 0;
}
