/**
 * @file WalkKickVariant.h
 *
 * This file declares information to create a InWalkKick.
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Representations/Configuration/KickInfo.h"
#include "RobotParts/Legs.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include "Tools/Motion/KickPrecision.h"
#include "Tools/Motion/WalkKickType.h"
#include <memory>

STREAMABLE(DelayKickParams,
{,
  (float)(0.f) delay, // how much delay shall be done before the kick (in s)?
  (int)(-1) kickIndex, // for which step shall the delay be applied?
  (bool)(false) forceDelay,
});

STREAMABLE(WalkKickVariant,
{
  WalkKickVariant() = default;
  WalkKickVariant(const KickInfo::KickType kickType, const WalkKicks::Type walkKickType, const Legs::Leg kickLeg,
                  const KickPrecision precision, const float power, const Angle direction, const bool shiftTurnKickPose);

  bool operator==(const WalkKickVariant& other) const;

  ENUM(DiagonalKickState,
  {,
    none,
    allowed,
    set,
  });

  STREAMABLE(DiagonalKickInfo,
  {,
    (WalkKickVariant::DiagonalKickState)(WalkKickVariant::DiagonalKickState::none) diagonalKickState,
    (Vector2f)(Vector2f::Zero()) contactPoint,
  }),

  (KickInfo::KickType)(KickInfo::walkForwardsLeft) kickType,
  (WalkKicks::Type)(WalkKicks::none) walkKickType,
  (Legs::Leg)(Legs::left) kickLeg,
  (KickPrecision)(KickPrecision::notPrecise) precision,
  (float)(0.f) length, /**< The length of the kick. */
  (float)(0.f) power,
  (Angle)(0_deg) direction,
  (float)(0.f) kickInterpolation, // interpolation factor between forward and turn kick
  (bool)(false) shiftTurnKickPose,
  (WalkKickVariant::DiagonalKickInfo) diagonalKickInfo,
  (float)(-1.f) ballEstimationTime, // ball estimation time (used to propagate the ball position)
  (DelayKickParams) delayParams,
  (int)(0) kickIndex,
});

inline WalkKickVariant::WalkKickVariant(const KickInfo::KickType kickType, const WalkKicks::Type walkKickType,
                                        const Legs::Leg kickLeg, const KickPrecision precision, const float length, const Angle direction, const bool shiftTurnKickPose) :
  kickType(kickType), walkKickType(walkKickType), kickLeg(kickLeg), precision(precision), length(length), direction(direction), shiftTurnKickPose(shiftTurnKickPose) {}

inline bool WalkKickVariant::operator==(const WalkKickVariant& other) const
{
  return kickType == other.kickType && kickLeg == other.kickLeg;
}
