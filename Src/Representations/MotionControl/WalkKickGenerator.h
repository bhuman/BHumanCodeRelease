/**
 * @file WalkKickGenerator.h
 *
 * This file declares a representation that generates walk kicks.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/KickInfo.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Tools/Function.h"
#include "Tools/Motion/WalkKickType.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <memory>

STREAMABLE(WalkKickVariant,
{
  WalkKickVariant() = default;
  WalkKickVariant(KickInfo::KickType kickType, WalkKicks::Type walkKickType, Legs::Leg kickLeg, float power, Angle direction);

  bool operator==(const WalkKickVariant& other) const,

  (KickInfo::KickType)(KickInfo::walkForwardsLeft) kickType,
  (WalkKicks::Type)(WalkKicks::none) walkKickType,
  (Legs::Leg)(Legs::left) kickLeg,
  (float)(1.f) power,
  (Angle)(0_deg) direction,
  (float)(0.f) kickInterpolation, // interpolation factor between forward and turn kick
});

inline WalkKickVariant::WalkKickVariant(KickInfo::KickType kickType, WalkKicks::Type walkKickType, Legs::Leg kickLeg, float power, Angle direction) : kickType(kickType), walkKickType(walkKickType), kickLeg(kickLeg), power(power), direction(direction) {}

inline bool WalkKickVariant::operator==(const WalkKickVariant& other) const
{
  return kickType == other.kickType && kickLeg == other.kickLeg;
}

STREAMABLE(WalkKickGenerator,
{
  FUNCTION(bool(const WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange, const bool alignPrecisely, const bool preStepAllowed, const bool turnKickAllowed, const float kickPoseShiftY)) canStart; /**< Checks whether the specified kick can start in the upcoming phase. */
  FUNCTION(std::unique_ptr<MotionPhase>(const WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange, const bool playSound, const float kickPoseShiftY)) createPhase; /**< Creates a phase to execute the specified kick. */
  FUNCTION(Pose2f(const bool isLeftPhase, const Angle direction)) getVShapeWalkStep;
  FUNCTION(void(const Pose2f& step)) drawStep,
});
