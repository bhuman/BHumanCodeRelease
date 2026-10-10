/**
 * @file KickInfo.h
 * Declaration of a representation that contains information about the available kicks
 * @author Thijs Jeffry de Haas
 * @author Lukas Post
 */

#pragma once

#include "Math/Angle.h"
#include "Tools/Motion/MotionPhase.h"
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
    /** Dummy helper kicks. */
    walkForwardStealBallLeft,
    walkForwardStealBallRight,

    /** New RL kicks for booster robots. */
    rlKickDynamic,
    rlKickDynamicOld,
    rlKickStrong,

    // newKick is a placeholder
    newKick,
  });

  STREAMABLE(Kick,
  {,
    (KickInfo::KickType) kickType, /**< The name of the kick. */
    (Angle) rotationOffset, /**< The offset of the kick pose, e.g. how much does the robot need to be rotated away additionally from the kick angle. */
    (Vector2f) ballOffset, /**< The offset of the kick pose, e.g. the robot pose relative to the ball. */
    (Rangef) range, /**< The kick range (in mm). Note: all kicks deviate, this is just an approximation. */
    (Rangef) ballVelocity, /**< The velocity of the ball after the kick (in mm/s). Note with this value the range value is determined at the start of the software. */
    (unsigned) executionTime, /**< Time needed to perform the kick or extra penalty for deficient kicks (in ms) */
    (Angle) postRotationOffset, /**< The angle at which the ball should be after the kick has been performed (relative to the pose of the robot after the kick, because some kicks change the rotation of the robot) */
    (MotionPhase::Type) motion, /**< Is the kick a walk kick or a KickEngine kick? */
    (Legs::Leg) kickLeg, /**< Which leg is the kicking one? TODO should be removed ... */
    (bool) mirror, /**< Mirror the kick? TODO should be removed ... */
  });

  inline KickInfo::KickType mirror(const KickInfo::KickType kick) const
  {
    switch(kick)
    {
      // Intentionally left out walkForwardStealBallLeft and walkForwardStealBallRight
      case KickInfo::rlKickDynamic:
        return KickInfo::rlKickDynamic;
      case KickInfo::rlKickDynamicOld:
        return KickInfo::rlKickDynamicOld;
      case KickInfo::rlKickStrong:
        return KickInfo::rlKickStrong;
      default:
      {
        FAIL("Unknown kick type.");
        return KickInfo::rlKickDynamic;
      }
    }
  };

  const KickInfo::Kick& operator[](const KickInfo::KickType kickType) const { return kicks[kickType]; },

  (ENUM_INDEXED_ARRAY(Kick, KickType)) kicks,
});
