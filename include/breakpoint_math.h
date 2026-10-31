#pragma once

#include <math.h>
#include <stdint.h>

enum class BrakePointApproach : uint8_t {
  Inactive = 0,
  FromBelow = 1,
  FromAbove = 2
};

enum class BrakePointMode : uint8_t {
  Disabled = 0,
  Wall = 1,
  BlockIncrease = 2,
  BlockDecrease = 3
};

inline float bpNormalize360(float angle) {
  float result = fmodf(angle, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result;
}

inline float bpShortestErrorDeg(float target, float current) {
  float error = bpNormalize360(target) - bpNormalize360(current);
  if (error > 180.0f) {
    error -= 360.0f;
  } else if (error < -180.0f) {
    error += 360.0f;
  }
  return error;
}

inline float bpForwardDistanceDeg(float fromAngleDeg, float toAngleDeg) {
  return bpNormalize360(toAngleDeg - fromAngleDeg);
}

inline float bpBackwardDistanceDeg(float fromAngleDeg, float toAngleDeg) {
  return bpNormalize360(fromAngleDeg - toAngleDeg);
}

inline float bpDirectionalDistanceDeg(float currentAngleDeg,
                                      float targetAngleDeg,
                                      int8_t logicalDirection) {
  return logicalDirection >= 0
             ? bpForwardDistanceDeg(currentAngleDeg, targetAngleDeg)
             : bpBackwardDistanceDeg(currentAngleDeg, targetAngleDeg);
}

inline float bpPredictedLeadDistanceDeg(float speedDps,
                                        uint16_t reactionMs,
                                        uint16_t decelDps2,
                                        float leadMarginDeg,
                                        float minLeadDeg,
                                        float maxLeadDeg) {
  speedDps = fabsf(speedDps);
  const float reactionDistance =
      speedDps * (static_cast<float>(reactionMs) / 1000.0f);
  const float deceleration = decelDps2 > 0
                                 ? static_cast<float>(decelDps2)
                                 : 1.0f;
  const float stoppingDistance =
      (speedDps * speedDps) / (2.0f * deceleration);
  float lead = reactionDistance + stoppingDistance + leadMarginDeg;
  if (lead < minLeadDeg) {
    lead = minLeadDeg;
  }
  if (lead > maxLeadDeg) {
    lead = maxLeadDeg;
  }
  return lead;
}

inline bool bpDirectionForbidden(BrakePointApproach approach,
                                 int8_t logicalDirection) {
  if (approach == BrakePointApproach::FromBelow) {
    return logicalDirection > 0;
  }
  if (approach == BrakePointApproach::FromAbove) {
    return logicalDirection < 0;
  }
  return false;
}

inline BrakePointApproach bpFixedApproachForMode(BrakePointMode mode) {
  if (mode == BrakePointMode::BlockIncrease) {
    return BrakePointApproach::FromBelow;
  }
  if (mode == BrakePointMode::BlockDecrease) {
    return BrakePointApproach::FromAbove;
  }
  return BrakePointApproach::Inactive;
}

inline bool bpModeDirectionForbidden(BrakePointMode mode,
                                     BrakePointApproach wallApproach,
                                     int8_t logicalDirection) {
  if (mode == BrakePointMode::BlockIncrease) {
    return logicalDirection > 0;
  }
  if (mode == BrakePointMode::BlockDecrease) {
    return logicalDirection < 0;
  }
  if (mode == BrakePointMode::Wall) {
    return bpDirectionForbidden(wallApproach, logicalDirection);
  }
  return false;
}

inline bool bpSegmentCrossed(float previousAngleDeg,
                             float currentAngleDeg,
                             float brakePointDeg,
                             BrakePointApproach &approach) {
  const float delta = bpShortestErrorDeg(currentAngleDeg, previousAngleDeg);
  constexpr float epsilon = 0.0001f;
  if (delta > epsilon) {
    const float forwardDistance =
        bpNormalize360(brakePointDeg - previousAngleDeg);
    if (forwardDistance <= delta + epsilon) {
      approach = BrakePointApproach::FromBelow;
      return true;
    }
  } else if (delta < -epsilon) {
    const float backwardDistance =
        bpNormalize360(previousAngleDeg - brakePointDeg);
    if (backwardDistance <= -delta + epsilon) {
      approach = BrakePointApproach::FromAbove;
      return true;
    }
  }
  return false;
}

inline bool bpSegmentCrossedForMode(BrakePointMode mode,
                                    float previousAngleDeg,
                                    float currentAngleDeg,
                                    float brakePointDeg,
                                    BrakePointApproach &approach) {
  BrakePointApproach crossedFrom = BrakePointApproach::Inactive;
  if (!bpSegmentCrossed(previousAngleDeg,
                        currentAngleDeg,
                        brakePointDeg,
                        crossedFrom)) {
    return false;
  }
  if (mode == BrakePointMode::Wall) {
    approach = crossedFrom;
    return true;
  }
  const BrakePointApproach fixed = bpFixedApproachForMode(mode);
  if (fixed == crossedFrom) {
    approach = fixed;
    return true;
  }
  return false;
}

inline bool bpPathCrossesForbidden(BrakePointMode mode,
                                   float startAngleDeg,
                                   float targetAngleDeg,
                                   float brakePointDeg) {
  const float delta = bpShortestErrorDeg(targetAngleDeg, startAngleDeg);
  constexpr float epsilon = 0.0001f;
  if (fabsf(delta) <= epsilon || mode == BrakePointMode::Disabled) {
    return false;
  }
  const int8_t direction = delta > 0.0f ? 1 : -1;
  BrakePointApproach approach = bpFixedApproachForMode(mode);
  if (mode == BrakePointMode::Wall) {
    approach = direction > 0 ? BrakePointApproach::FromBelow
                             : BrakePointApproach::FromAbove;
  }
  if (!bpModeDirectionForbidden(mode, approach, direction)) {
    return false;
  }
  const float distance =
      bpDirectionalDistanceDeg(startAngleDeg, brakePointDeg, direction);
  return distance <= fabsf(delta) + epsilon;
}

inline bool bpOriginalSideDistance(BrakePointApproach approach,
                                   float brakePointDeg,
                                   float currentAngleDeg,
                                   float &distanceDeg) {
  const float signedError =
      bpShortestErrorDeg(brakePointDeg, currentAngleDeg);
  if (approach == BrakePointApproach::FromBelow && signedError >= 0.0f) {
    distanceDeg = signedError;
    return true;
  }
  if (approach == BrakePointApproach::FromAbove && signedError <= 0.0f) {
    distanceDeg = -signedError;
    return true;
  }
  distanceDeg = 0.0f;
  return false;
}
