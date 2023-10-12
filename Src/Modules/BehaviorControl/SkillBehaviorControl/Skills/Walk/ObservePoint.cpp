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
  Vector2f vectorToLeftObstacle;
  Vector2f vectorToRightObstacle;

  option(ObservePoint)
  {
    Vector2f clippedTarget = theRobotPose * p.target;
    theFieldDimensions.clipToField(clippedTarget);
    clippedTarget = theRobotPose.inverse() * clippedTarget;
    const float clippedTargetSqrNorm = clippedTarget.squaredNorm();

    const Pose2f allowedSpeed(1.f, 1.f, 0.7f);

    initial_state(far)
    {
      transition
      {
        if(!calculateVectorsToBlockingObstacles(clippedTarget, 5_deg))
        {
          if(clippedTargetSqrNorm < sqr(closeBoundary))
          {
            left = clippedTarget.y() > 0.f;
            goto close;
          }
          else if(clippedTargetSqrNorm < sqr(farBoundary))
            goto mid;
        }
        else
        {
          Angle angleBetweenLeftAndTarget = clippedTarget.angleTo(vectorToLeftObstacle);
          Angle angleBetweenRightAndTarget = clippedTarget.angleTo(vectorToRightObstacle);
          if(angleBetweenLeftAndTarget > angleBetweenRightAndTarget)
          {
            goto targetBlockedByObstacleMoveRight;
          }
          else
          {
            goto targetBlockedByObstacleMoveLeft;
          }
        }
      }

      action
      {
        theWalkToPointSkill({.target = clippedTarget,
                             .speed = allowedSpeed,
                             .reduceWalkingSpeed = !theGameState.isCornerKick()});
        theLookAtPointSkill({.target = (Vector3f() << clippedTarget, 0.f).finished()});
      }
    }

    state(mid)
    {
      transition
      {
        if(!calculateVectorsToBlockingObstacles(clippedTarget, 5_deg))
        {
          if(clippedTargetSqrNorm < sqr(closeBoundary))
          {
            left = clippedTarget.y() > 0.f;
            goto close;
          }
          else if(clippedTargetSqrNorm > sqr(farBoundary + hysteresisOffset))
            goto far;
        }
        else
        {
          Angle angleBetweenLeftAndTarget = clippedTarget.angleTo(vectorToLeftObstacle);
          Angle angleBetweenRightAndTarget = clippedTarget.angleTo(vectorToRightObstacle);
          if(angleBetweenLeftAndTarget > angleBetweenRightAndTarget)
          {
            goto targetBlockedByObstacleMoveRight;
          }
          else
          {
            goto targetBlockedByObstacleMoveLeft;
          }
        }
      }
      action
      {
        theWalkToPointSkill({.target = {clippedTarget.angle()},
                             .speed = allowedSpeed,
                             .reduceWalkingSpeed = !theGameState.isCornerKick(),
                             .rough = true});
        theLookLeftAndRightSkill({.startLeft = clippedTarget.y() > 0.f, .maxPan = 40_deg, .speed = 60_deg});
      }
    }

    state(close)
    {
      transition
      {
        if(!calculateVectorsToBlockingObstacles(clippedTarget, 5_deg))
        {
          if(clippedTargetSqrNorm > sqr(closeBoundary + hysteresisOffset))
            goto mid;
          else if(clippedTargetSqrNorm > sqr(farBoundary + 300.f))
            goto far;
        }
        else
        {
          Angle angleBetweenLeftAndTarget = clippedTarget.angleTo(vectorToLeftObstacle);
          Angle angleBetweenRightAndTarget = clippedTarget.angleTo(vectorToRightObstacle);
          if(angleBetweenLeftAndTarget > angleBetweenRightAndTarget)
          {
            goto targetBlockedByObstacleMoveRight;
          }
          else
          {
            goto targetBlockedByObstacleMoveLeft;
          }
        }
      }

      action
      {
        theWalkToPointSkill({.target = {left ? 90_deg : -90_deg},
                             .speed = allowedSpeed,
                             .reduceWalkingSpeed = !theGameState.isCornerKick(),
                             .rough = true});
        theLookForwardSkill();
      }
    }

    state(targetBlockedByObstacleMoveRight)
    {
      transition
      {
        if(!calculateVectorsToBlockingObstacles(clippedTarget, 15_deg))
        {
          if(clippedTargetSqrNorm < sqr(closeBoundary))
            goto close;
          else if(clippedTargetSqrNorm > sqr(closeBoundary + hysteresisOffset))
            goto mid;
          else if(clippedTargetSqrNorm > sqr(farBoundary + hysteresisOffset))
            goto far;
        }
        else
        {
          Angle angleBetweenLeftAndTarget = clippedTarget.angleTo(vectorToLeftObstacle);
          Angle angleBetweenRightAndTarget = clippedTarget.angleTo(vectorToRightObstacle);
          if(4.f * angleBetweenLeftAndTarget < angleBetweenRightAndTarget)
            goto targetBlockedByObstacleMoveLeft;
        }
      }
      action
      {
        if(vectorToRightObstacle.squaredNorm() < sqr(1500.f))
          theWalkToPointSkill({.target = {vectorToRightObstacle.rotate(-30_deg)},
                               .speed = allowedSpeed,
                               .reduceWalkingSpeed = !theGameState.isCornerKick(),
                               .rough = true});
        else
          theWalkToPointSkill({.target = {vectorToRightObstacle.rotate(-10_deg)},
                               .speed = allowedSpeed,
                               .reduceWalkingSpeed = !theGameState.isCornerKick(),
                               .rough = true});
        theLookAtPointSkill({.target = (Vector3f() << clippedTarget, 0.f).finished()});
      }
    }

    state(targetBlockedByObstacleMoveLeft)
    {
      transition
      {
        if(!calculateVectorsToBlockingObstacles(clippedTarget, 15_deg))
        {
          if(clippedTargetSqrNorm < sqr(closeBoundary))
            goto close;
          else if(clippedTargetSqrNorm > sqr(closeBoundary + hysteresisOffset))
            goto mid;
          else if(clippedTargetSqrNorm > sqr(farBoundary + hysteresisOffset))
            goto far;
        }
        else
        {
          Angle angleBetweenLeftAndTarget = clippedTarget.angleTo(vectorToLeftObstacle);
          Angle angleBetweenRightAndTarget = clippedTarget.angleTo(vectorToRightObstacle);
          if(angleBetweenLeftAndTarget > 4.f * angleBetweenRightAndTarget)
            goto targetBlockedByObstacleMoveRight;
        }
      }
      action
      {
        if(vectorToLeftObstacle.squaredNorm() < sqr(1500.f))
          theWalkToPointSkill({.target = {vectorToLeftObstacle.rotate(30_deg)},
                               .speed = allowedSpeed,
                               .reduceWalkingSpeed = !theGameState.isCornerKick(),
                               .rough = true});
        else
          theWalkToPointSkill({.target = {vectorToLeftObstacle.rotate(10_deg)},
                               .speed = allowedSpeed,
                               .reduceWalkingSpeed = !theGameState.isCornerKick(),
                               .rough = true});
        theLookAtPointSkill({.target = (Vector3f() << clippedTarget, 0.f).finished()});
      }
    }
  }

  bool left = false;

  bool calculateVectorsToBlockingObstacles(const Vector2f& clippedTarget, float angleToCheck)
  {
    bool isBlocked = false;
    bool hasAngleChanged = true;
    vectorToLeftObstacle = clippedTarget.rotated(angleToCheck);
    vectorToRightObstacle = clippedTarget.rotated(-angleToCheck);

    for(std::size_t i = 0; i < theObstacleModel.obstacles.size() && hasAngleChanged; i++)
    {
      hasAngleChanged = false;
      for(const Obstacle& obstacle : theObstacleModel.obstacles)
      {
        if(vectorToLeftObstacle.squaredNorm() > obstacle.center.squaredNorm() + hysteresisOffset &&
           vectorToLeftObstacle.angle() > obstacle.right.angle() &&
           vectorToRightObstacle.angle() < obstacle.left.angle())
        {
          if(vectorToLeftObstacle.angle() < obstacle.left.angle() || !isBlocked)
          {
            vectorToLeftObstacle = Vector2f(obstacle.left).rotate(10_deg).normalize(clippedTarget.norm());
            hasAngleChanged = true;
          }
          if(vectorToRightObstacle.angle() > obstacle.right.angle() || !isBlocked)
          {
            vectorToRightObstacle = Vector2f(obstacle.right).rotate(-10_deg).normalize(clippedTarget.norm());
            hasAngleChanged = true;
          }
          isBlocked = true;
        }
      }
    }

    return isBlocked;
  }
};

MAKE_SKILL_IMPLEMENTATION(ObservePointImpl);
