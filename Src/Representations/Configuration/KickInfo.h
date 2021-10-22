/**
 * @file KickInfo.h
 * Declaration of a representation that contains information about the available kicks
 * @author jeff
 * @author Lukas Post
 */

#pragma once

#include "Representations/MotionControl/KickRequest.h"
#include "Tools/Math/Angle.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Motion/WalkKickType.h"
#include "Tools/Range.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * @struct KickInfo
 * A representation that contains the nearest pose for shooting a goal
 */
STREAMABLE(KickInfo,
{
  ENUM(KickType,
  {,
    forwardFastRight,
    forwardFastLeft,
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
    walkForwardStealBallLeft,
    walkForwardStealBallRight,
    walkForwardsRightAlternative,
    walkForwardsLeftAlternative,

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

  const KickInfo::Kick& operator[](const KickInfo::KickType kickType) const { return kicks[kickType]; },

  (ENUM_INDEXED_ARRAY(Kick, KickType)) kicks,
});
