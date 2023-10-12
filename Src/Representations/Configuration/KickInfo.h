/**
 * @file KickInfo.h
 * Declaration of a representation that contains information about the available kicks
 * @author jeff
 * @author Lukas Post
 */

#pragma once

#include "Representations/MotionControl/KickRequest.h"
#include "Math/Angle.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Motion/WalkKickType.h"
#include "Math/Range.h"
#include "RobotParts/Legs.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include "Streaming/EnumIndexedArray.h"

/**
 * @struct KickInfo
 * A representation that contains information about the available kicks
 */
STREAMABLE(KickInfo,
{
  ENUM(KickType,
  {,
    forwardFastRight,
    forwardFastLeft,
    forwardFastRightPass,
    forwardFastLeftPass,
    forwardFastRightLong,
    forwardFastLeftLong,
    walkForwardsRight,
    walkForwardsLeft,
    walkForwardsRightLong,
    walkForwardsLeftLong,
    walkSidewardsRightFootToRight,
    walkSidewardsLeftFootToLeft,
    walkTurnRightFootToLeft,
    walkTurnLeftFootToRight,
    walkTurnRightFootToLeftShifted,
    walkTurnLeftFootToRightShifted,
    walkForwardStealBallLeft,
    walkForwardStealBallRight,
    walkForwardsRightAlternative,
    walkForwardsLeftAlternative,
    walkForwardsRightVeryLong,
    walkForwardsLeftVeryLong,

    // kicks up to here are used by the KickView
    newKick,
  });

  STREAMABLE(Kick,
  {,
    (KickInfo::KickType) kickType,
    (Angle) rotationOffset,
    (Vector2f) ballOffset,
    (Rangef) range,
    (Rangef) ballVelocity,
    (Rangef) exclusionRange,
    (unsigned) executionTime, /**< Time needed to perform the kick or extra penalty for deficient kicks (in ms) */
    (Angle) postRotationOffset, /**< The angle at which the ball should be after the kick has been performed (relative to the pose of the robot after the kick, because some kicks change the rotation of the robot) */
    (MotionPhase::Type) motion,
    (KickRequest::KickMotionID) kickMotionType,
    (WalkKicks::Type) walkKickType,
    (Legs::Leg) kickLeg,
    (bool) mirror,
  });

  inline KickInfo::KickType mirror(const KickInfo::KickType kick) const
  {
    switch(kick)
    {
      case KickInfo::forwardFastRight:
        return KickInfo::forwardFastLeft;
      case KickInfo::forwardFastLeft:
        return KickInfo::forwardFastRight;
      case KickInfo::forwardFastRightPass:
        return KickInfo::forwardFastLeftPass;
      case KickInfo::forwardFastLeftPass:
        return KickInfo::forwardFastRightPass;
      case KickInfo::forwardFastRightLong:
        return KickInfo::forwardFastLeftLong;
      case KickInfo::forwardFastLeftLong:
        return KickInfo::forwardFastRightLong;
      case KickInfo::walkForwardsRight:
        return KickInfo::walkForwardsLeft;
      case KickInfo::walkForwardsLeft:
        return KickInfo::walkForwardsRight;
      case KickInfo::walkForwardsRightLong:
        return KickInfo::walkForwardsLeftLong;
      case KickInfo::walkForwardsLeftLong:
        return KickInfo::walkForwardsRightLong;
      case KickInfo::walkSidewardsRightFootToRight:
        return KickInfo::walkSidewardsLeftFootToLeft;
      case KickInfo::walkSidewardsLeftFootToLeft:
        return KickInfo::walkSidewardsRightFootToRight;
      case KickInfo::walkTurnRightFootToLeft:
        return KickInfo::walkTurnLeftFootToRight;
      case KickInfo::walkTurnLeftFootToRight:
        return KickInfo::walkTurnRightFootToLeft;
      case KickInfo::walkTurnRightFootToLeftShifted:
        return KickInfo::walkTurnLeftFootToRightShifted;
      case KickInfo::walkTurnLeftFootToRightShifted:
        return KickInfo::walkTurnRightFootToLeftShifted;
      case KickInfo::walkForwardStealBallLeft:
        return KickInfo::walkForwardStealBallRight;
      case KickInfo::walkForwardStealBallRight:
        return KickInfo::walkForwardStealBallLeft;
      case KickInfo::walkForwardsRightAlternative:
        return KickInfo::walkForwardsLeftAlternative;
      case KickInfo::walkForwardsLeftAlternative:
        return KickInfo::walkForwardsRightAlternative;
      case KickInfo::walkForwardsLeftVeryLong:
        return KickInfo::walkForwardsRightVeryLong;
      case KickInfo::walkForwardsRightVeryLong:
        return KickInfo::walkForwardsLeftVeryLong;
      default:
      {
        FAIL("Unknown kick type.");
        return KickInfo::walkForwardsLeft;
      }
    }
  };

  const KickInfo::Kick& operator[](const KickInfo::KickType kickType) const { return kicks[kickType]; },

  (ENUM_INDEXED_ARRAY(Kick, KickType)) kicks,
});
