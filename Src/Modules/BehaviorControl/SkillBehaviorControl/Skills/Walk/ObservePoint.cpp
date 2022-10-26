/**
 * @file ObservePoint.cpp
 *
 * This file implements an implementation of the ObservePoint skill.
 *
 * @author Nicole Schrader
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "MathBase/BHMath.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Debugging/Debugging.h"

SKILL_IMPLEMENTATION(ObservePointImpl,
{,
  IMPLEMENTS(ObservePoint),
  CALLS(LookAtPoint),
  CALLS(LookForward),
  CALLS(LookLeftAndRight),
  CALLS(WalkToPoint),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  REQUIRES(ObstacleModel),
  DEFINES_PARAMETERS(
  {,
    (float)(300.f) closeBoundary,
    (float)(1200.f) farBoundary,
    (float)(300.f) hysteresisOffset,
  }),
});

class ObservePointImpl : public ObservePointImplBase
{
  option(ObservePoint)
  {
    Vector2f clippedTarget = theRobotPose * p.target;
    theFieldDimensions.clipToField(clippedTarget);
    clippedTarget = theRobotPose.inversePose * clippedTarget;
    const float clippedTargetSqrNorm = clippedTarget.squaredNorm();

    common_transition
    {
      for(const Obstacle& obstacle : theObstacleModel.obstacles)
      {
        const Angle angleOfHalfObstacle = 3.f * obstacle.center.angleTo(obstacle.left);
        if(clippedTarget.squaredNorm() > obstacle.center.squaredNorm() &&
           clippedTarget.angleTo(obstacle.center) < angleOfHalfObstacle)
          goto targetBlockedByObstacle;
      }
    }

    initial_state(far)
    {
      transition
      {
        if(clippedTargetSqrNorm < sqr(closeBoundary))
        {
          left = clippedTarget.y() > 0.f;
          goto close;
        }
        else if(clippedTargetSqrNorm < sqr(farBoundary))
          goto mid;
      }

      action
      {
        theWalkToPointSkill({.target = clippedTarget, .reduceWalkingSpeed = !theGameState.isCornerKick()});
        theLookAtPointSkill({.target = (Vector3f() << clippedTarget, 0.f).finished()});
      }
    }

    state(mid)
    {
      transition
      {
        if(clippedTargetSqrNorm < sqr(closeBoundary))
        {
          left = clippedTarget.y() > 0.f;
          goto close;
        }
        else if(clippedTargetSqrNorm > sqr(farBoundary + hysteresisOffset))
          goto far;
      }
      action
      {
        theWalkToPointSkill({.target = Pose2f(clippedTarget.angle()), .reduceWalkingSpeed = !theGameState.isCornerKick(), .rough = true});
        theLookLeftAndRightSkill({.startLeft = clippedTarget.y() > 0.f, .maxPan = 40_deg, .speed = 60_deg});
      }
    }

    state(close)
    {
      transition
      {
        if(clippedTargetSqrNorm > sqr(closeBoundary + hysteresisOffset))
          goto mid;
        else if(clippedTargetSqrNorm > sqr(farBoundary + 300.f))
          goto far;
      }

      action
      {
        theWalkToPointSkill({.target = Pose2f(left ? 90_deg : -90_deg), .reduceWalkingSpeed = !theGameState.isCornerKick(), .rough = true});
        theLookForwardSkill();
      }
    }

    state(targetBlockedByObstacle)
    {
      transition
      {
        if(clippedTargetSqrNorm < sqr(closeBoundary))
          goto close;
        else if(clippedTargetSqrNorm > sqr(closeBoundary + hysteresisOffset))
          goto mid;
        else if(clippedTargetSqrNorm > sqr(farBoundary + 300.f))
          goto far;
      }

      action
      {
        Angle angleToLeftMostObstacle;
        Angle angleToRightMostObstacle;
        Vector2f vectorToLeftMostObstacle = Vector2f::Zero();
        Vector2f vectorToRightMostObstacle = Vector2f::Zero();
        for(const Obstacle& obstacle : theObstacleModel.obstacles)
        {
          const Angle angleOfHalfObstacle = 3.f * obstacle.center.angleTo(obstacle.left);
          if(clippedTarget.squaredNorm() > obstacle.center.squaredNorm() &&
             clippedTarget.angleTo(obstacle.center) < angleOfHalfObstacle)
          {
            const Angle angleToLeftObstacle = clippedTarget.angleTo(obstacle.left);
            const Angle angleToRightObstacle = clippedTarget.angleTo(obstacle.right);
            if(angleToLeftObstacle > angleToLeftMostObstacle && angleToRightObstacle > angleToRightMostObstacle)
            {
              angleToLeftMostObstacle = angleToLeftObstacle;
              angleToRightMostObstacle = angleToRightObstacle;
              vectorToLeftMostObstacle = obstacle.left;
              vectorToRightMostObstacle = obstacle.right;
            }
            else if(angleToLeftObstacle < angleToRightObstacle && angleToRightObstacle > angleToRightMostObstacle)
            {
              angleToRightMostObstacle = angleToRightObstacle;
              vectorToRightMostObstacle = obstacle.right;
            }
            else if(angleToRightObstacle < angleToLeftObstacle && angleToLeftObstacle > angleToLeftMostObstacle)
            {
              angleToLeftMostObstacle = angleToLeftObstacle;
              vectorToLeftMostObstacle = obstacle.left;
            }
          }
        }
        if(angleToLeftMostObstacle < angleToRightMostObstacle)
        {
          theWalkToPointSkill({ .target = Pose2f(vectorToLeftMostObstacle.rotate(1)), .reduceWalkingSpeed = !theGameState.isCornerKick(), .rough = true });
          theLookAtPointSkill({ .target = (Vector3f() << clippedTarget, 0.f).finished() });
        }
        else
        {
          theWalkToPointSkill({ .target = Pose2f(vectorToRightMostObstacle.rotate(-1)), .reduceWalkingSpeed = !theGameState.isCornerKick(), .rough = true });
          theLookAtPointSkill({ .target = (Vector3f() << clippedTarget, 0.f).finished() });
        }
      }
    }
  }

  bool left = false;
};

MAKE_SKILL_IMPLEMENTATION(ObservePointImpl);
