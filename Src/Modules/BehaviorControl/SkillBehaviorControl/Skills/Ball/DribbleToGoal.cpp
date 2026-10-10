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
       args((KickPrecision) alignPrecisely),
       load((Rangef) minKickRange, /**< Planned look ahead dribble kick range. */
            (float) lookActiveMinBallDistance, /**< If the ball is at least this far away, use lookActive with withBall = true. */
            (float) awayRange,
            (float) closeRange))
{
  const float useMinKickRange = mapToRange(theFieldBall.positionOnField.x(), theFieldDimensions.xPosOpponentPenaltyArea * 0.5f, theFieldDimensions.xPosOpponentGoalArea, std::min(minKickRange.max, theFieldDimensions.xPosOpponentGoalArea - theFieldDimensions.xPosOpponentPenaltyArea * 0.5f), minKickRange.min);

  initial_state(away)
  {
    transition
    {
      if(theFieldBall.positionRelative.squaredNorm() < sqr(closeRange))
        goto close;
    }
    action
    {
      GoToBallAndDribble({.targetDirection = -theRobotPose.rotation, .alignPrecisely = alignPrecisely});
    }
  }

  state(close)
  {
    transition
    {
      if(theFieldBall.positionRelative.squaredNorm() > sqr(awayRange))
        goto away;
    }
    action
    {
      const Angle dribbleAngle = Angle::normalize(theDribbleTarget.calculateDribbleAngle(theFieldBall.positionOnField) - theRobotPose.rotation);
      // TODO set kickLength to something higher?
      GoToBallAndDribble({.targetDirection = dribbleAngle,
                          .alignPrecisely = alignPrecisely,
                          .kickLength = useMinKickRange,
                          .lookActiveWithBall = theFieldBall.positionRelative.squaredNorm() > sqr(lookActiveMinBallDistance)});
    }
  }
}
