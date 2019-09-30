/**
 * @file TimeToReachBall.cpp
 *
 * This file implements the implementation of the TimeToReachBall skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamSkills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"

TEAM_SKILL_IMPLEMENTATION(TimeToReachBallImpl,
{,
  IMPLEMENTS(TimeToReachBall),
  REQUIRES(LibCheck),
  MODIFIES(TeamBehaviorStatus),
});

class TimeToReachBallImpl : public TimeToReachBallImplBase
{
  void execute(const TimeToReachBall& p) override
  {
    theTeamBehaviorStatus.timeToReachBall = p.timeToReachBall;
    theLibCheck.inc(LibCheck::timeToReachBall);
  }
};

MAKE_TEAM_SKILL_IMPLEMENTATION(TimeToReachBallImpl);
