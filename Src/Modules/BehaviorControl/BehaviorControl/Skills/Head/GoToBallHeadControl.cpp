/**
 * @file GoToBallHeadControl.cpp
 *
 * This file implements an implementation of the GoToBallHeadControl skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(GoToBallHeadControlImpl,
{,
  CALLS(LookActive),
  CALLS(LookAtBall),
  CALLS(LookAtPoint),
  REQUIRES(BallModel),
  REQUIRES(FieldBall),
  REQUIRES(ObstacleModel),
  IMPLEMENTS(GoToBallHeadControl),
  DEFINES_PARAMETERS(
  {,
    (float)(250.f) lookAtBallDistanceLow, /**< If the distance to the target becomes smaller than this, the head looks at the ball. */
    (float)(310.f) lookAtBallDistanceHigh, /**< If the distance to the target becomes larger than this, other things than the ball may be looked at. */
    (float)(500.f) mayLookAtKickTargetDistanceLow, /**< If the distance to the target becomes smaller than this, the head may look at the kick target. */
    (float)(600.f) mayLookAtKickTargetDistanceHigh, /**< If the distance to the target becomes larger than this, the head control switches back to look active. */
    (float)(400.f) areaAroundBallRadiusLow, /**< If an obstacle is in the circle around the ball with this radius, the head control switches back to look active. */
    (float)(600.f) areaAroundBallRadiusHigh, /**< If no obstacle is in the circle around the ball with this radius, the head looks at the kick target direction. */
  }),
});

class GoToBallHeadControlImpl : public GoToBallHeadControlImplBase
{
  option(GoToBallHeadControl)
  {
    initial_state(lookActive)
    {
      transition
      {
        if(p.distanceToTarget < lookAtBallDistanceLow)
          goto lookAtBall;
        else if(shouldLookAtKickTarget(p, false))
          goto lookAtKickTarget;
      }

      action
      {
        theLookActiveSkill(/* withBall: */ true);
      }
    }

    state(lookAtKickTarget)
    {
      transition
      {
        if(p.distanceToTarget < lookAtBallDistanceLow)
          goto lookAtBall;
        else if(!shouldLookAtKickTarget(p, true))
          goto lookActive;
      }

      action
      {
        theLookAtPointSkill((Vector3f() << p.kickTargetRelative, 0.f).finished());
      }
    }

    state(lookAtBall)
    {
      transition
      {
        if(p.distanceToTarget > lookAtBallDistanceHigh)
        {
          if(shouldLookAtKickTarget(p, false))
            goto lookAtKickTarget;
          else
            goto lookActive;
        }
      }
      action
      {
        theLookAtBallSkill();
      }
    }
  }

  bool shouldLookAtKickTarget(const GoToBallHeadControl& p, bool didLookAtKickTarget) const
  {
    // Do not do this if the calling option does not allow it.
    if(!p.lookAtKickTarget)
      return false;
    // Do this only when sufficiently close to the target.
    if(p.distanceToTarget > (didLookAtKickTarget ? mayLookAtKickTargetDistanceHigh : mayLookAtKickTargetDistanceLow))
      return false;
    // If the kick target is too far to the side or even behind, do not attempt to look there.
    if(std::abs(p.kickTargetRelative.angle()) > (didLookAtKickTarget ? 90_deg : 60_deg))
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
  }
};

MAKE_SKILL_IMPLEMENTATION(GoToBallHeadControlImpl);
