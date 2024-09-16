/**
 * @file GoToBallAndDribble.cpp
 *
 * This file implements the GoToBallAndDribble skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) GoToBallAndDribble,
       args((Angle) targetDirection,
            (KickPrecision) alignPrecisely,
            (float) kickLength,
            (bool) lookActiveWithBall,
            (PreStepType) preStepType,
            (bool) turnKickAllowed,
            (const Rangea&) directionPrecision),
       defs((float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
            (float)(900.f) switchToLibWalkDistance)) /**< If the target is closer than this distance, LibWalk is used. */
{
  const Pose2f dribblePose(targetDirection, theFieldInterceptBall.interceptedEndPositionRelative);

  if(lookActiveWithBall)
    LookActive({.withBall = true,
                .onlyOwnBall = true});
  else
    GoToBallHeadControl({.distanceToTarget = dribblePose.translation.norm()});

  initial_state(dribbleFarRange)
  {
    transition
    {
      if(dribblePose.translation.squaredNorm() < sqr(switchToLibWalkDistance))
        goto dribbleCloseRange;
    }

    action
    {
      PublishMotion({.target = dribblePose.translation});
      auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * dribblePose, {1.f, 1.f, 1.f});
      Dribble({.targetDirection = targetDirection,
               .obstacleAvoidance = obstacleAvoidance,
               .alignPrecisely = alignPrecisely,
               .kickLength = kickLength,
               .preStepType = preStepType,
               .turnKickAllowed = turnKickAllowed,
               .directionPrecision = directionPrecision});
    }
  }

  state(dribbleCloseRange)
  {
    transition
    {
      if(dribblePose.translation.squaredNorm() > sqr(switchToPathPlannerDistance))
        goto dribbleFarRange;
    }

    action
    {
      PublishMotion({.target = dribblePose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(dribblePose, /* rough: */ true, /* disableObstacleAvoidance: */ false, /* toBall: */ true);
      Dribble({.targetDirection = targetDirection,
               .obstacleAvoidance = obstacleAvoidance,
               .alignPrecisely = alignPrecisely,
               .kickLength = kickLength,
               .preStepType = preStepType,
               .turnKickAllowed = turnKickAllowed,
               .directionPrecision = directionPrecision});
    }
  }
}
