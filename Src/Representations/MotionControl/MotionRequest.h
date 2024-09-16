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
#include "Math/Angle.h"
#include "Math/Geometry.h"
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Tools/Motion/KickPrecision.h"
#include "Tools/Motion/PreStepType.h"

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
    dive, /**< Execute a diving motion. */
    special, /**< Execute a special motion. */
    replayWalk, /**< Replay a recorded walkPhase history. */
    photoMode, /**< Photo mode, to knead the robot in custom positions. */
  });

  STREAMABLE(ObstacleAvoidance,
  {
    STREAMABLE(PathSegment,
    {,
      (Geometry::Circle) obstacle, /**< The obstacle circle around which to walk. */
      (bool) clockwise, /**< Whether the obstacle should be circumnavigated in clockwise direction (else counterclockwise). */
    });

    /**
     * This operator is only needed to determine, whether the default parameter was passed to a skill.
     * It does not really check equality, but is sufficuent for comparing with an empty object.
     * @param other The object this one is compared to.
     * @return Are they equal?
     */
    bool operator==(const ObstacleAvoidance& other) const
    {
      ASSERT(other.avoidance.isZero() && other.path.empty());
      return avoidance == other.avoidance && path.size() == other.path.size();
    },

    (Vector2f)(Vector2f::Zero()) avoidance, /**< A vector that steers away from close obstacles. */
    (std::vector<PathSegment>) path, /**< The path of obstacles that should be followed. */
  });

  struct Dive
  {
    ENUM(Request,
    {,
      prepare,
      jumpLeft,
      jumpRight,
      squatArmsBackLeft,
      squatArmsBackRight,
      squatWideArmsBackLeft,
      squatWideArmsBackRight,
    });
  };

  struct Special
  {
    ENUM(Request,
    {,
      demoBannerWave,
      demoBannerWaveInitial,
    });
  };

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
  (std::optional<Vector2f>) targetOfInterest, /**< If filled, the first entry is used to decide how the body shall be orientated while walking, so the camera can see this target at all times. Target is in relative coordinates. */
  (bool)(false) forceSideWalking, /**< If set, the robot is forced to side walk. */
  (bool)(false) shouldInterceptBall, /**< If the ball should be intercepted. */
  (bool)(false) shouldWalkOutOfBallLine, /**< If the robot should intenionally leave the ball line*/

  (Angle)(0_deg) targetDirection, /**< The target direction of the ball (only used if type is walkToBallAndKick or dribble). */
  (Rangea) directionPrecision, /**< The min and max target direction difference of the ball (only used if type is walkToBallAndKick or dribble). */
  (KickInfo::KickType)(KickInfo::forwardFastLeft) kickType, /**< The kick type (only used if type is walkToBallAndKick). */
  (float)(0.f) kickLength, /**< The kick length (only used if type is walkToBallAndKick) (in mm). */
  (KickPrecision)(KickPrecision::notPrecise) alignPrecisely, /**< Whether the robot should align (and kick) more precisely than usual, probably taking more time. */
  (PreStepType)(PreStepType::allowed) preStepType, /**< Is the InWalkKick allowed to have a preStep? */
  (bool)(true) turnKickAllowed, /**< Can the forward InWalkKick be executed with rotation? */
  (bool)(false) shiftTurnKickPose, /**< Should the turn kick be shifted? */

  (Dive::Request)(Dive::prepare) diveRequest, /**< The specific requested diving motion (only used if type is dive). */
  (Special::Request)(Special::demoBannerWave) specialRequest, /**< The specific requested special motion (only used if type is special). */

  (Pose2f) odometryData, /**< The odometry data when this request was created. */
  (BallState) ballEstimate, /**< The most recent ball estimate. */
  (unsigned)(0) ballEstimateTimestamp, /**< The timestamp when the ball estimate was calculated. */
  (unsigned)(0) ballTimeWhenLastSeen, /**< The timestamp when the ball has been seen. */
});
