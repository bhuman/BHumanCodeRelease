/**
 * @file WalkToPointObstacle.cpp
 *
 * This file implements an implementation for the WalkToPointObstacle skill.
 * Obstacles at the target position result in a new adjusted target position, to avoid running into another robot.
 * Afterwards WalkToPoint with the new target position is executed
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"
#include "Debugging/DebugDrawings.h"
#include "Math/Geometry.h"
#include <cmath>

option((SkillBehaviorControl) WalkToPointObstacle,
       args((const Pose2f&) target,
            (const Pose2f&) speed,
            (ReduceWalkSpeedType) reduceWalkingSpeed,
            (bool) rough,
            (bool) disableObstacleAvoidance,
            (bool) disableAligning,
            (bool) disableStanding,
            (bool) disableAvoidFieldBorder,
            (const std::optional<Vector2f>&) targetOfInterest,
            (bool) forceSideWalking),
       defs((float)(500.f) switchToRequestedPoseDistance, /**< Update the shifted target pose if the new one differs by more than this value to the old one. */
            (float)(1000.f) minDistanceToLeaveForAvoidance, /**< If the target is further away than this threshold, leave the obstacle avoidance state. */
            (float)(600.f) minDistanceToTargetForAvoidance, /**< Start obstacle avoidance at this distance to the target. */
            (float)(100.f) ignoreObstacleAvoidanceDistance, /**< Set obstacleAvoidance for walkToPointSkill to false at this distance to the target. */
            (float)(400.f) positionOffsetIfOccupied, /**< The radius around an obstacle that occupies my target position. */
            (Angle)(22.5_deg) deleteObstacleCircleRange), /**< A previous obstacle is only deleted if it is outside +- this angle. */
       vars((Geometry::Circle)({}) lastCircle,
            (std::optional<Pose2f>)({}) shiftedTarget,
            (Pose2f)({}) targetWhenShifted))
{
  const auto calcShiftedPose = [&](const Pose2f& target, const Geometry::Circle& circle)
  {
    targetWhenShifted = target;
    const Vector2f relativeObstacleShiftVector = circle.center.normalized(circle.radius);
    std::vector<Vector2f> evalPoints;
    evalPoints.push_back(circle.center - relativeObstacleShiftVector);
    evalPoints.push_back(circle.center - relativeObstacleShiftVector.rotated(90_deg));
    evalPoints.push_back(circle.center - relativeObstacleShiftVector.rotated(-90_deg));

    shiftedTarget = target;
    float bestRating = -1;
    for(const auto& vec : evalPoints)
    {
      const float rating = theExpectedGoals.getRating(theRobotPose * vec, false);
      if(rating > bestRating)
      {
        shiftedTarget->translation = vec;
        bestRating = rating;
      }
    }
    shiftedTarget->translation = theRobotPose * shiftedTarget->translation;
    shiftedTarget->rotation += theRobotPose.rotation;
  };

  // Calculates the circle around an obstacle which occupies my target position.
  const auto getObstacleAtMyPositionCircle = [&](const Vector2f& position) -> Geometry::Circle
  {
    CIRCLE("option:WalkToPointObstacle:obstacleAtMyPosition", lastCircle.center.x(), lastCircle.center.y(), lastCircle.radius, 20, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA::black);
    const Vector2f posRel(theRobotPose.inverse() * position);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      if((obstacle.center - posRel).squaredNorm() < sqr(positionOffsetIfOccupied))
        return lastCircle = Geometry::Circle(theRobotPose * obstacle.center, positionOffsetIfOccupied);

    if((lastCircle.center - position).squaredNorm() < sqr(lastCircle.radius) &&
       std::abs((theRobotPose.inverse() * lastCircle.center).angle()) > deleteObstacleCircleRange)
      return lastCircle;

    lastCircle.radius = 0.f;
    return lastCircle;
  };

  auto circle = getObstacleAtMyPositionCircle(theRobotPose * target.translation);
  circle.center = theRobotPose.inverse() * circle.center;
  bool ignoreObstacle = false;

  if(shiftedTarget)
  {
    if((targetWhenShifted.translation - target.translation).squaredNorm() > sqr(switchToRequestedPoseDistance))
    {
      if(circle.radius == 0.f || circle.center.squaredNorm() > sqr(circle.radius))
        ignoreObstacle = true;
      else
        calcShiftedPose(target, circle);
    }
    else
      shiftedTarget->rotation = target.rotation + theRobotPose.rotation;
  }

  common_transition
  {
    if(disableObstacleAvoidance || ignoreObstacle)
      goto walkFar;
  }

  initial_state(walkFar)
  {
    transition
    {
      if(!disableObstacleAvoidance && target.translation.squaredNorm() < sqr(minDistanceToTargetForAvoidance) && circle.radius > 0.f && circle.center.squaredNorm() < sqr(circle.radius))
        goto avoidObstacle;
      if(action_done)
        goto targetReached;
    }
    action
    {
      WalkToPoint({.target = target,
                   .speed = speed,
                   .reduceWalkingSpeed = reduceWalkingSpeed,
                   .rough = rough,
                   .disableObstacleAvoidance = disableObstacleAvoidance,
                   .disableAligning = disableAligning,
                   .disableStanding = disableStanding,
                   .disableAvoidFieldBorder = disableAvoidFieldBorder,
                   .targetOfInterest = targetOfInterest,
                   .forceSideWalking = forceSideWalking});
    }
  }

  state(avoidObstacle)
  {
    transition
    {
      if(!disableObstacleAvoidance && target.translation.squaredNorm() > sqr(minDistanceToLeaveForAvoidance))
        goto walkFar;
      if(action_done)
        goto targetReached;
    }
    action
    {
      if(state_time == 0)
        calcShiftedPose(target, circle);
      const Pose2f relativeTarget = theRobotPose.inverse() * *shiftedTarget;
      WalkToPoint({.target = relativeTarget,
                   .speed = speed,
                   .reduceWalkingSpeed = reduceWalkingSpeed,
                   .rough = rough,
                   .disableObstacleAvoidance = disableObstacleAvoidance
                                               || relativeTarget.translation.squaredNorm() <= sqr(ignoreObstacleAvoidanceDistance),
                   .disableAligning = disableAligning,
                   .disableStanding = disableStanding,
                   .disableAvoidFieldBorder = disableAvoidFieldBorder,
                   .targetOfInterest = targetOfInterest,
                   .forceSideWalking = forceSideWalking});
    }
  }

  target_state(targetReached)
  {
    transition
    {
      if(!action_done)
        goto walkFar;
    }
    action
    {
      const Pose2f relativeTarget = shiftedTarget ? theRobotPose.inverse() * *shiftedTarget : target;
      WalkToPoint({.target = relativeTarget,
                   .speed = speed,
                   .reduceWalkingSpeed = reduceWalkingSpeed,
                   .rough = rough,
                   .disableObstacleAvoidance = disableObstacleAvoidance
                                               || relativeTarget.translation.squaredNorm() <= sqr(ignoreObstacleAvoidanceDistance),
                   .disableAligning = disableAligning,
                   .disableStanding = disableStanding,
                   .disableAvoidFieldBorder = disableAvoidFieldBorder,
                   .targetOfInterest = targetOfInterest,
                   .forceSideWalking = forceSideWalking});
    }
  }
}
