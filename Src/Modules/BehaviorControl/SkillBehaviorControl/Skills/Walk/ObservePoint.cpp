/**
 * @file ObservePoint.cpp
 *
 * This file implements the ObservePoint skill.
 *
 * @author Nicole Schrader
 */

#include "SkillBehaviorControl.h"
#include "MathBase/BHMath.h"

option((SkillBehaviorControl) ObservePoint,
       args((const Vector2f&) target),
       defs((float)(300.f) closeBoundary,
            (float)(1200.f) farBoundary,
            (float)(300.f) hysteresisOffset),
       vars((bool)(false) left))
{
  Vector2f vectorToLeftObstacle;
  Vector2f vectorToRightObstacle;

  const auto calculateVectorsToBlockingObstacles = [&](const Vector2f& clippedTarget, float angleToCheck)
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
  };

  Vector2f clippedTarget = theRobotPose * target;
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
      WalkToPoint({.target = clippedTarget,
                   .speed = allowedSpeed,
                   .reduceWalkingSpeed = theGameState.isCornerKick() ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow});
      LookAtPoint({.target = (Vector3f() << clippedTarget, 0.f).finished()});
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
      WalkToPoint({.target = {clippedTarget.angle()},
                   .speed = allowedSpeed,
                   .reduceWalkingSpeed = theGameState.isCornerKick() ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow,
                   .rough = true});
      LookLeftAndRight({.startLeft = clippedTarget.y() > 0.f,
                        .maxPan = 40_deg,
                        .speed = 60_deg});
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
      WalkToPoint({.target = {left ? 90_deg : -90_deg},
                   .speed = allowedSpeed,
                   .reduceWalkingSpeed = theGameState.isCornerKick() ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow,
                   .rough = true});
      LookForward();
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
      WalkToPoint({.target = {vectorToRightObstacle.rotate(vectorToRightObstacle.squaredNorm() < sqr(1500.f) ? -30_deg : -10_deg)},
                   .speed = allowedSpeed,
                   .reduceWalkingSpeed = theGameState.isCornerKick() ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow,
                   .rough = true});
      LookAtPoint({.target = (Vector3f() << clippedTarget, 0.f).finished()});
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
      WalkToPoint({.target = {vectorToLeftObstacle.rotate(vectorToLeftObstacle.squaredNorm() < sqr(1500.f) ? 30_deg : 10_deg)},
                   .speed = allowedSpeed,
                   .reduceWalkingSpeed = theGameState.isCornerKick() ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow,
                   .rough = true});
      LookAtPoint({.target = (Vector3f() << clippedTarget, 0.f).finished()});
    }
  }
}
