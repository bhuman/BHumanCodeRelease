/**
 * @file DribbleToGoal.cpp
 *
 * This file implements a card that dribbles the ball to the goal.
 * It is always runnable for a striker.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Debugging/DebugDrawings.h"
#include <map>

option((SkillBehaviorControl) DribbleToGoal,
       defs((float)(1000.f) dribbleRange, /**< Planned look ahead dribble kick range. */
            (float)(500.f) lookActiveMinBallDistance)) /**< If the ball is at least this far away, use lookActive with withBall = true. */
{
  common_transition
  {
    if(theFieldBall.positionRelative.squaredNorm() > sqr(500.f))
      goto away;
    else
      goto close;
  }

  initial_state(away)
  {
    action
    {
      GoToBallAndDribble({.targetDirection = -theRobotPose.rotation});
    }
  }

  state(close)
  {
    action
    {
      const Angle dribbleAngle = Angle::normalize(theDribbleTarget.calculateDribbleAngle(theFieldInterceptBall.interceptedEndPositionOnField) - theRobotPose.rotation);
      // TODO set kickLength to something higher?
      GoToBallAndDribble({.targetDirection = dribbleAngle,
                          .kickLength = dribbleRange,
                          .lookActiveWithBall = theFieldInterceptBall.interceptedEndPositionRelative.squaredNorm() > sqr(lookActiveMinBallDistance)});
    }
  }
}
