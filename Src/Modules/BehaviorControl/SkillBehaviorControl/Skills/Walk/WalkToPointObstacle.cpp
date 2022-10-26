/**
 * @file WalkToPointObstacle.cpp
 *
 * This file implements an implementation for the WalkToPointObstacle skill.
 * Obstacles at the target position result in a new adjusted target position, to avoid running into another robot.
 * Afterwards WalkToPoint with the new target position is executed
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(WalkToPointObstacleImpl,
{,
  IMPLEMENTS(WalkToPointObstacle),
  REQUIRES(ExpectedGoals),
  REQUIRES(FrameInfo),
  REQUIRES(LibPosition),
  REQUIRES(RobotPose),
  CALLS(WalkToPoint),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) switchToRequestedPoseDistance, /**< Update the shifted target pose if the new one differs by more than this value to the old one. */
    (float)(1000.f) minDistanceToLeaveForAvoidance, /**< If the target is further away than this threshold, leave the obstacle avoidance state. */
    (float)(600.f) minDistanceToTargetForAvoidance, /**< Start obstacle avoidance at this distance to the target. */
    (float)(100.f) ignoreObstacleAvoidanceDistance, /**< Set obstacleAvoidance for walkToPointSkill to false at this distance to the target. */
    (float)(400.) minDistanceToObstacle,
  }),
});

class WalkToPointObstacleImpl : public WalkToPointObstacleImplBase
{
  option(WalkToPointObstacle)
  {
    auto circle = theLibPosition.getObstacleAtMyPositionCircle(theRobotPose * p.target.translation);
    circle.center = theRobotPose.inversePose * circle.center;
    bool ignoreObstacle = false;
    if(shiftedTarget)
    {
      if((targetWhenShifted.translation - p.target.translation).squaredNorm() > sqr(switchToRequestedPoseDistance))
      {
        if(circle.radius == 0.f || circle.center.squaredNorm() > sqr(minDistanceToObstacle))
          ignoreObstacle = true;
        else
          calcShiftedPose(p.target, circle);
      }
      else
        shiftedTarget->rotation = p.target.rotation + theRobotPose.rotation;
    }

    common_transition
    {
      if(p.disableObstacleAvoidance || ignoreObstacle)
        goto walkFar;
    }

    initial_state(walkFar)
    {
      transition
      {
        if(!p.disableObstacleAvoidance && p.target.translation.squaredNorm() < sqr(minDistanceToTargetForAvoidance) && circle.radius > 0.f && circle.center.squaredNorm() < sqr(minDistanceToObstacle))
          goto avoidObstacle;
        if(theWalkToPointSkill.isDone())
          goto targetReached;
      }
      action
      {
        shiftedTarget.reset();
        theWalkToPointSkill(p.target, p.speed, p.reduceWalkingSpeed, p.rough, p.disableObstacleAvoidance,
                            p.disableAligning, p.disableStanding, p.disableAvoidFieldBorder,
                            p.targetOfInterest, p.forceSideWalking);
      }
    }

    state(avoidObstacle)
    {
      transition
      {
        if(!p.disableObstacleAvoidance && p.target.translation.squaredNorm() > sqr(minDistanceToLeaveForAvoidance))
          goto walkFar;
        if(theWalkToPointSkill.isDone())
          goto targetReached;
      }
      action
      {
        if(state_time == 0)
          calcShiftedPose(p.target, circle);
        const Pose2f relativeTarget = theRobotPose.inversePose * *shiftedTarget;
        theWalkToPointSkill(relativeTarget, p.speed, p.reduceWalkingSpeed, p.rough, relativeTarget.translation.squaredNorm() > sqr(ignoreObstacleAvoidanceDistance) ? p.disableObstacleAvoidance : true,
                            p.disableAligning, p.disableStanding, p.disableAvoidFieldBorder,
                            p.targetOfInterest, p.forceSideWalking);
      }
    }

    target_state(targetReached)
    {
      transition
      {
        if(!theWalkToPointSkill.isDone())
          goto walkFar;
      }
      action
      {
        const Pose2f relativeTarget = shiftedTarget ? theRobotPose.inversePose * *shiftedTarget : p.target;
        theWalkToPointSkill(relativeTarget, p.speed, p.reduceWalkingSpeed, p.rough, relativeTarget.translation.squaredNorm() > sqr(ignoreObstacleAvoidanceDistance) ? p.disableObstacleAvoidance : true,
                            p.disableAligning, p.disableStanding, p.disableAvoidFieldBorder,
                            p.targetOfInterest, p.forceSideWalking);
      }
    }
  }

  void calcShiftedPose(const Pose2f& target, const Geometry::Circle& circle)
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
      const float rating = theExpectedGoals.getRating(theRobotPose * vec);
      if(rating > bestRating)
      {
        shiftedTarget->translation = vec;
        bestRating = rating;
      }
    }
    shiftedTarget->translation = theRobotPose * shiftedTarget->translation;
    shiftedTarget->rotation += theRobotPose.rotation;
  }

  std::optional<Pose2f> shiftedTarget;
  Pose2f targetWhenShifted;
};

MAKE_SKILL_IMPLEMENTATION(WalkToPointObstacleImpl);
