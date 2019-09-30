/**
 * @file TeamActivity.cpp
 *
 * This file implements the implementation of the TeamActivity skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamSkills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"

TEAM_SKILL_IMPLEMENTATION(TeamActivityImpl,
{,
  IMPLEMENTS(TeamActivity),
  REQUIRES(LibCheck),
  MODIFIES(TeamBehaviorStatus),
});

class TeamActivityImpl : public TeamActivityImplBase
{
  void execute(const TeamActivity& p) override
  {
    theTeamBehaviorStatus.teamActivity = p.teamActivity;
    theLibCheck.inc(LibCheck::teamActivity);
  }
};

MAKE_TEAM_SKILL_IMPLEMENTATION(TeamActivityImpl);
