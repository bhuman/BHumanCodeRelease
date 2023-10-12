/**
 * @file Block.cpp
 *
 * This file implements an implementation of the Block skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Math/Angle.h"
#include "Math/Range.h"
#include <algorithm>
#include <cmath>

SKILL_IMPLEMENTATION(BlockImpl,
{,
  IMPLEMENTS(Block),
  CALLS(KeyFrameArms),
  CALLS(LookActive),
  CALLS(LookLeftAndRight),
  CALLS(WalkToPoint),
  REQUIRES(CameraInfo),
  REQUIRES(FieldBall),
  DEFINES_PARAMETERS(
  {,
    (float)(300.f) distanceToTarget, /**< The reference distance to the target. */
    (float)(200.f) overrideArmsDistance, /**< When closer than this to the block target, the arms are not taken back anymore. */
    (float)(400.f) ignoreObstaclesDistance, /**< When closer than this to the block target, obstacle avoidance is disabled. */
    (float)(70.f) halfRobotWidth, /**< The half of the robot width that is used to determine coverage of the ball from the target. */
    (Angle)(7_deg) angleDifferenceThreshold, /**< Threshold to control when a blocking robot starts to track the blocked robot. */
  }),
});

class BlockImpl : public BlockImplBase
{
  void execute(const Block& p) override
  {
    ballRelative = p.useAlternativeBall ? p.alternativeBall : theFieldBall.recentBallPositionRelative();
    // If the block target is not in front of me, I am not blocking anymore.
    if(blocking && std::abs(p.target.angle()) > 45_deg)
      blocking = false;
    // If I do not cover the ball from the block target's perspective, I am not blocking anymore.
    if(blocking)
    {
      const Angle ballAngleFromBlockedObstacle = (ballRelative - p.target).angle();
      const Angle leftAngleFromBlockedObstacle = (Vector2f(0.f, halfRobotWidth) - p.target).angle();
      const Angle rightAngleFromBlockedObstacle = (Vector2f(0.f, -halfRobotWidth) - p.target).angle();
      // This works because at this point it is known that the target is in front of me.
      if(!Rangef(leftAngleFromBlockedObstacle, rightAngleFromBlockedObstacle).isInside(ballAngleFromBlockedObstacle))
        blocking = false;
    }

    if(!blocking)
    {
      const Pose2f blockPoseRelative = calcBlockPose(p.target, distanceToTarget);
      const float blockPoseDistanceSqr = blockPoseRelative.translation.squaredNorm();
      if(blockPoseDistanceSqr < sqr(overrideArmsDistance))
        theKeyFrameArmsSkill({.motion = ArmKeyFrameRequest::useDefault});
      theLookActiveSkill({.ignoreBall = true});
      theWalkToPointSkill({.target = blockPoseRelative,
                           .rough = true,
                           .disableObstacleAvoidance = blockPoseDistanceSqr < sqr(ignoreObstaclesDistance),
                           .disableStanding = true});
      if(theWalkToPointSkill.isDone())
        blocking = true;
      p.setState("walkToBlockPose");
    }
    else
    {
      // Override arm obstacle avoidance.
      theKeyFrameArmsSkill({.motion = ArmKeyFrameRequest::useDefault});
      // This only works as long as upper and lower camera have the same opening angles and are y-wise in the center of the head.
      const Angle maxPan = std::min(std::abs(p.target.angle() + theCameraInfo.openingAngleWidth / 3.f), std::abs(p.target.angle() - theCameraInfo.openingAngleWidth / 3.f));
      theLookLeftAndRightSkill({.startLeft = true,
                                .maxPan = maxPan,
                                .tilt = 23_deg,
                                .speed = 60_deg});

      Pose2f trackingPose;
      if(p.target.squaredNorm() > sqr(distanceToTarget))
        trackingPose = calcBlockPose(p.target, distanceToTarget);
      else if(std::abs(Angle::normalize((-p.target).angle() - (ballRelative - p.target).angle())) > angleDifferenceThreshold)
        trackingPose = calcBlockPose(p.target, p.target.norm());
      // Else the trackingPose stays 0.
      theWalkToPointSkill({.target = trackingPose,
                           .rough = true,
                           .disableObstacleAvoidance = true,
                           .disableStanding = true});
      p.setState("blocking");
    }
  }

  void reset(const Block&) override
  {
    blocking = false;
  }

  /**
   * Calculates the block pose.
   * @param target The relative position of the block target.
   * @param distance The desired distance to the block target.
   * @return The block pose.
   */
  Pose2f calcBlockPose(const Vector2f target, float distance) const
  {
    const Vector2f position = target - (target - ballRelative).normalized(distance);
    return Pose2f((target - position).angle(), position);
  }

  bool blocking; /**< Whether the robot is current blocking or still walking to the block pose. */
  Vector2f ballRelative; /**< The relative ball position. */
};

MAKE_SKILL_IMPLEMENTATION(BlockImpl);
