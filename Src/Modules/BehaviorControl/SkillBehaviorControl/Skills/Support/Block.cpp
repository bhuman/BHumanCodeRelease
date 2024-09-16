/**
 * @file Block.cpp
 *
 * This file implements the Block skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include <algorithm>
#include <cmath>

option((SkillBehaviorControl) Block,
       args((const Vector2f&) target,
            (bool) useAlternativeBall,
            (Vector2f) alternativeBall),
       defs((float)(300.f) distanceToTarget, /**< The reference distance to the target. */
            (float)(200.f) overrideArmsDistance, /**< When closer than this to the block target, the arms are not taken back anymore. */
            (float)(400.f) ignoreObstaclesDistance, /**< When closer than this to the block target, obstacle avoidance is disabled. */
            (float)(70.f) halfRobotWidth, /**< The half of the robot width that is used to determine coverage of the ball from the target. */
            (Angle)(7_deg) angleDifferenceThreshold)) /**< Threshold to control when a blocking robot starts to track the blocked robot. */
{
  // The relative ball position.
  const Vector2f ballRelative = useAlternativeBall ? alternativeBall : theFieldBall.recentBallPositionRelative();


  // Calculates the block pose from the relative position of the block target and
  // the desired distance to the block target.
  const auto calcBlockPose = [&](const Vector2f& target, float distance)
  {
    const Vector2f position = target - (target - ballRelative).normalized(distance);
    return Pose2f((target - position).angle(), position);
  };

  initial_state(notBlocking)
  {

    transition
    {
      if(state_time > 0 && action_done)
        goto blocking;
    }
    action
    {
      const Pose2f blockPoseRelative = calcBlockPose(target, distanceToTarget);
      const float blockPoseDistanceSqr = blockPoseRelative.translation.squaredNorm();
      if(blockPoseDistanceSqr < sqr(overrideArmsDistance))
        KeyFrameArms({.motion = ArmKeyFrameRequest::useDefault});
      LookActive({.ignoreBall = true});
      WalkToPoint({.target = blockPoseRelative,
                   .rough = true,
                   .disableObstacleAvoidance = blockPoseDistanceSqr < sqr(ignoreObstaclesDistance),
                   .disableStanding = true});
    }
  }

  state(blocking)
  {
    transition
    {
      // If the block target is not in front of me, I am not blocking anymore.
      if(std::abs(target.angle()) > 45_deg)
        goto notBlocking;

      const Angle ballAngleFromBlockedObstacle = (ballRelative - target).angle();
      const Angle leftAngleFromBlockedObstacle = (Vector2f(0.f, halfRobotWidth) - target).angle();
      const Angle rightAngleFromBlockedObstacle = (Vector2f(0.f, -halfRobotWidth) - target).angle();

      // This works because at this point it is known that the target is in front of me.
      if(!Rangef(leftAngleFromBlockedObstacle, rightAngleFromBlockedObstacle).isInside(ballAngleFromBlockedObstacle))
        goto notBlocking;
    }
    action
    {
      // Override arm obstacle avoidance.
      KeyFrameArms({.motion = ArmKeyFrameRequest::useDefault});
      // This only works as long as upper and lower camera have the same opening angles and are y-wise in the center of the head.
      const Angle maxPan = std::min(std::abs(target.angle() + theCameraInfo.openingAngleWidth / 3.f), std::abs(target.angle() - theCameraInfo.openingAngleWidth / 3.f));
      LookLeftAndRight({.startLeft = true,
                        .maxPan = maxPan,
                        .tilt = 23_deg,
                        .speed = 60_deg});

      Pose2f trackingPose;
      if(target.squaredNorm() > sqr(distanceToTarget))
        trackingPose = calcBlockPose(target, distanceToTarget);
      else if(std::abs(Angle::normalize((-target).angle() - (ballRelative - target).angle())) > angleDifferenceThreshold)
        trackingPose = calcBlockPose(target, target.norm());
      // Else the trackingPose stays 0.
      WalkToPoint({.target = trackingPose,
                   .rough = true,
                   .disableObstacleAvoidance = true,
                   .disableStanding = true});
    }
  }
}
