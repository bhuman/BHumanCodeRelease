/**
 * @file MotionRequest.h
 *
 * This file declares a struct that represents the request of the behavior to the motion.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/KickInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(MotionRequest,
{
  ENUM(Motion,
  {,
    playDead, /**< Set all joints to zero stiffness. */
    stand, /**< Stand. */
    walkAtAbsoluteSpeed, /**< Walk at a given speed. */
    walkAtRelativeSpeed, /**< Walk at a given speed, given as a ratio of the maximum. */
    walkToPose, /**< Walk to a target pose. */
    walkToBallAndKick, /**< Walk to the ball and kick it. */
    dribble, /**< Dribble the ball. */
    getUp, /**< Get up. */
    keyframeMotion, /**< Execute a keyframe motion. */
    replayWalk, /**< Replay a recorded walkPhase history. */
  });

  STREAMABLE(ObstacleAvoidance,
  {
    STREAMABLE(PathSegment,
    {,
      (Geometry::Circle) obstacle, /**< The obstacle circle around which to walk. */
      (bool) clockwise, /**< Whether the obstacle should be circumnavigated in clockwise direction (else counterclockwise). */
    }),
    (Vector2f)(Vector2f::Zero()) avoidance, /**< A vector that steers away from close obstacles. */
    (std::vector<PathSegment>) path, /**< The path of obstacles that should be followed. */
  });

  bool isWalking() const
  {
    return motion == walkAtAbsoluteSpeed || motion == walkAtRelativeSpeed || motion == walkToPose || motion == walkToBallAndKick || motion == dribble || motion == replayWalk;
  }

  void draw() const;

  void verify() const,

  (Motion)(playDead) motion, /**< The requested motion type. */

  (bool)(false) standHigh, /**< Whether the robot should stretch its knees while standing (only used if type is stand). */

  (Pose2f) walkSpeed, /**< The walk speed, interpreted as (average) mm/s for walkAtAbsoluteSpeed or else as ratios of the maximum speed (only used if type is one of the walk* types or dribble). */
  (Pose2f) walkTarget, /**< The walk target (only used if type is walkToPose). */
  (bool)(false) keepTargetRotation, /**< Whether the target rotation should be reached as soon as possible, i.e. takes away one degree of freedom from motion (only used if type is walkToPose). */
  (ObstacleAvoidance) obstacleAvoidance, /**< Information about which obstacles there are to avoid (only used if type is walkTo* or dribble). */

  (Angle)(0_deg) targetDirection, /**< The target direction of the ball (only used if type is walkToBallAndKick or dribble). */
  (Rangea) directionPrecision, /**< The min and max target direction difference of the ball (only used if type is walkToBallAndKick or dribble). */
  (KickInfo::KickType)(KickInfo::forwardFastLeft) kickType, /**< The kick type (only used if type is walkToBallAndKick). */
  (float)(0.f) kickPower, /**< The kick power (only used if type is walkToBallAndKick). */
  (bool)(false) alignPrecisely, /**< Whether the robot should align (and kick) more precisely than usual, probably taking more time. */
  (bool)(true) preStepAllowed, /**< Is the InWalkKick allowed to have a preStep? */
  (bool)(true) turnKickAllowed, /**< Can the forward InWalkKick be executed with rotation? */

  (KeyframeMotionRequest) keyframeMotionRequest, /**< The specific requested keyframe motion (only used if type is keyframeMotion). */

  (Pose2f) odometryData, /**< The odometry data when this request was created. */
  (BallState) ballEstimate, /**< The most recent ball estimate. */
  (unsigned)(0) ballEstimateTimestamp, /**< The timestamp when the ball estimate was calculated. */
  (unsigned)(0) ballTimeWhenLastSeen, /**< The timestamp when the ball has been seen. */
});
