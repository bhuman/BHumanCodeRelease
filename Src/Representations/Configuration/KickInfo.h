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

    // newKick is a placeholder
    newKick,
  });

  STREAMABLE(Kick,
  {,
    (KickInfo::KickType) kickType, /**< The name of the kick. */
    (Angle) rotationOffset, /**< The offset of the kickpose, e.g. how much does the robot need to be rotated away additionally from the kick angle. */
    (Vector2f) ballOffset, /**< The offset of the kickpose, e.g. the robot pose relative to the ball. */
    (Rangef) range, /**< The kick range (in mm). Note: all kicks deviate, this is just an approximation. */
    (Rangef) ballVelocity, /**< The velocity of the ball after the kick (in mm/s). Note with this value the range value is determined at the start of the software. */
    (unsigned) executionTime, /**< Time needed to perform the kick or extra penalty for deficient kicks (in ms) */
    (Angle) postRotationOffset, /**< The angle at which the ball should be after the kick has been performed (relative to the pose of the robot after the kick, because some kicks change the rotation of the robot) */
    (MotionPhase::Type) motion, /**< Is the kick a walk kick or a KickEngine kick? */
    (KickRequest::KickMotionID) kickMotionType, /**< Kick name for the KickEngine. TODO should be removed ... */
    (WalkKicks::Type) walkKickType, /**< The name of the walk kick. */
    (Legs::Leg) kickLeg, /**< Which leg is the kicking one? TODO should be removed ... */
    (bool) mirror, /**< Mirror the kick? TODO should be removed ... */
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
