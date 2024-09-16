/**
 * @file GoToBallHeadControl.cpp
 *
 * This file implements the GoToBallHeadControl skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) GoToBallHeadControl,
       args((float) distanceToTarget,
            (bool) lookAtKickTarget,
            (Vector2f) kickTargetRelative),
       defs((float)(250.f) lookAtBallDistanceLow, /**< If the distance to the target becomes smaller than this, the head looks at the ball. */
            (float)(310.f) lookAtBallDistanceHigh, /**< If the distance to the target becomes larger than this, other things than the ball may be looked at. */
            (float)(500.f) mayLookAtKickTargetDistanceLow, /**< If the distance to the target becomes smaller than this, the head may look at the kick target. */
            (float)(600.f) mayLookAtKickTargetDistanceHigh, /**< If the distance to the target becomes larger than this, the head control switches back to look active. */
            (float)(400.f) areaAroundBallRadiusLow, /**< If an obstacle is in the circle around the ball with this radius, the head control switches back to look active. */
            (float)(600.f) areaAroundBallRadiusHigh)) /**< If no obstacle is in the circle around the ball with this radius, the head looks at the kick target direction. */
{
  const auto shouldLookAtKickTarget = [&](bool didLookAtKickTarget) -> bool
  {
    // Do not do this if the calling option does not allow it.
    if(!lookAtKickTarget)
      return false;
    // Do this only when sufficiently close to the target.
    if(distanceToTarget > (didLookAtKickTarget ? mayLookAtKickTargetDistanceHigh : mayLookAtKickTargetDistanceLow))
      return false;
    // If the kick target is too far to the side or even behind, do not attempt to look there.
    if(std::abs(kickTargetRelative.angle()) > (didLookAtKickTarget ? 90_deg : 60_deg))
      return false;
    // If the ball is moving, do not look the kick target.
    if(theBallModel.estimate.velocity.squaredNorm() > (didLookAtKickTarget ? sqr(50.f) : 0.f))
      return false;
    // Don't do it for too long if the ball isn't seen.
    if(!theFieldBall.ballWasSeen(didLookAtKickTarget ? 900 : 100))
      return false;
    // If there is any obstacle close to the ball it could move the ball, so the ball should be given priority.
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      if((obstacle.center - theFieldBall.positionRelative).squaredNorm() < sqr(didLookAtKickTarget ? areaAroundBallRadiusLow : areaAroundBallRadiusHigh))
        return false;
    // Otherwise, just do it.
    return true;
  };

  initial_state(lookActive)
  {
    transition
    {
      if(distanceToTarget < lookAtBallDistanceLow)
        goto lookAtBall;
      else if(shouldLookAtKickTarget(false))
        goto lookAtKickTarget;
    }

    action
    {
      LookActive({.withBall = true});
    }
  }

  state(lookAtKickTarget)
  {
    transition
    {
      if(distanceToTarget < lookAtBallDistanceLow)
        goto lookAtBall;
      else if(!shouldLookAtKickTarget(true))
        goto lookActive;
    }

    action
    {
      LookAtPoint({.target = (Vector3f() << kickTargetRelative, 0.f).finished()});
    }
  }

  state(lookAtBall)
  {
    transition
    {
      if(distanceToTarget > lookAtBallDistanceHigh)
      {
        if(shouldLookAtKickTarget(false))
          goto lookAtKickTarget;
        else
          goto lookActive;
      }
    }
    action
    {
      LookAtBall();
    }
  }
}
