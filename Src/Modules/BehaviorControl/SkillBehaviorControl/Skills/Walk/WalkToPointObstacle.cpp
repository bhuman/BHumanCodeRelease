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
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Debugging/DebugDrawings.h"
#include "Math/Geometry.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(WalkToPointObstacleImpl,
{,
  IMPLEMENTS(WalkToPointObstacle),
  REQUIRES(ExpectedGoals),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  CALLS(WalkToPoint),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) switchToRequestedPoseDistance, /**< Update the shifted target pose if the new one differs by more than this value to the old one. */
    (float)(1000.f) minDistanceToLeaveForAvoidance, /**< If the target is further away than this threshold, leave the obstacle avoidance state. */
    (float)(600.f) minDistanceToTargetForAvoidance, /**< Start obstacle avoidance at this distance to the target. */
    (float)(100.f) ignoreObstacleAvoidanceDistance, /**< Set obstacleAvoidance for walkToPointSkill to false at this distance to the target. */
    (float)(400.f) positionOffsetIfOccupied, /**< The radius around an obstacle that occupies my target position. */
    (Angle)(22.5_deg) deleteObstacleCircleRange, /**< A previous obstacle is only deleted if it is outside +- this angle. */
  }),
});

class WalkToPointObstacleImpl : public WalkToPointObstacleImplBase
{
  option(WalkToPointObstacle)
  {
    auto circle = getObstacleAtMyPositionCircle(theRobotPose * p.target.translation);
    circle.center = theRobotPose.inverse() * circle.center;
    bool ignoreObstacle = false;
    if(shiftedTarget)
    {
      if((targetWhenShifted.translation - p.target.translation).squaredNorm() > sqr(switchToRequestedPoseDistance))
      {
        if(circle.radius == 0.f || circle.center.squaredNorm() > sqr(circle.radius))
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
        if(!p.disableObstacleAvoidance && p.target.translation.squaredNorm() < sqr(minDistanceToTargetForAvoidance) && circle.radius > 0.f && circle.center.squaredNorm() < sqr(circle.radius))
          goto avoidObstacle;
        if(theWalkToPointSkill.isDone())
          goto targetReached;
      }
      action
      {
        shiftedTarget.reset();
        theWalkToPointSkill({.target = p.target,
                             .speed = p.speed,
                             .reduceWalkingSpeed = p.reduceWalkingSpeed,
                             .rough = p.rough,
                             .disableObstacleAvoidance = p.disableObstacleAvoidance,
                             .disableAligning = p.disableAligning,
                             .disableStanding = p.disableStanding,
                             .disableAvoidFieldBorder = p.disableAvoidFieldBorder,
                             .targetOfInterest = p.targetOfInterest,
                             .forceSideWalking = p.forceSideWalking});
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
        const Pose2f relativeTarget = theRobotPose.inverse() * *shiftedTarget;
        theWalkToPointSkill({.target = relativeTarget,
                             .speed = p.speed,
                             .reduceWalkingSpeed = p.reduceWalkingSpeed,
                             .rough = p.rough,
                             .disableObstacleAvoidance = p.disableObstacleAvoidance
                                                         || relativeTarget.translation.squaredNorm() <= sqr(ignoreObstacleAvoidanceDistance),
                             .disableAligning = p.disableAligning,
                             .disableStanding = p.disableStanding,
                             .disableAvoidFieldBorder = p.disableAvoidFieldBorder,
                             .targetOfInterest = p.targetOfInterest,
                             .forceSideWalking = p.forceSideWalking});
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
        const Pose2f relativeTarget = shiftedTarget ? theRobotPose.inverse() * *shiftedTarget : p.target;
        theWalkToPointSkill({.target = relativeTarget,
                             .speed = p.speed,
                             .reduceWalkingSpeed = p.reduceWalkingSpeed,
                             .rough = p.rough,
                             .disableObstacleAvoidance = p.disableObstacleAvoidance
                                                         || relativeTarget.translation.squaredNorm() <= sqr(ignoreObstacleAvoidanceDistance),
                             .disableAligning = p.disableAligning,
                             .disableStanding = p.disableStanding,
                             .disableAvoidFieldBorder = p.disableAvoidFieldBorder,
                             .targetOfInterest = p.targetOfInterest,
                             .forceSideWalking = p.forceSideWalking});
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

  void preProcess() override {}
  void preProcess(const WalkToPointObstacle&) override
  {
    DECLARE_DEBUG_DRAWING("skill:WalkToPointObstacle:obstacleAtMyPosition", "drawingOnField");
  }


  /**
   * Calculates the circle around an obstacle which occupies my target position.
   * @param position The potential target position on the field
   * @return The circle with the obstacle as center or radius 0 if none
   */
  Geometry::Circle getObstacleAtMyPositionCircle(const Vector2f& position)
  {
    CIRCLE("skill:WalkToPointObstacle:obstacleAtMyPosition", lastCircle.center.x(), lastCircle.center.y(), lastCircle.radius, 20, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA::black);
    const Vector2f posRel(theRobotPose.inverse() * position);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      if((obstacle.center - posRel).squaredNorm() < sqr(positionOffsetIfOccupied))
        return lastCircle = Geometry::Circle(theRobotPose * obstacle.center, positionOffsetIfOccupied);

    if((lastCircle.center - position).squaredNorm() < sqr(lastCircle.radius) &&
       std::abs((theRobotPose.inverse() * lastCircle.center).angle()) > deleteObstacleCircleRange)
      return lastCircle;

    lastCircle.radius = 0.f;
    return lastCircle;
  }

  Geometry::Circle lastCircle;

  std::optional<Pose2f> shiftedTarget;
  Pose2f targetWhenShifted;
};

MAKE_SKILL_IMPLEMENTATION(WalkToPointObstacleImpl);
