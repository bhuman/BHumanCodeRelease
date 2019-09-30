/**
 * @file TeammateRoles.cpp
 *
 * This file implements the implementation of the TeammateRoles skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamSkills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"

TEAM_SKILL_IMPLEMENTATION(TeammateRolesImpl,
{,
  IMPLEMENTS(TeammateRoles),
  REQUIRES(LibCheck),
  MODIFIES(TeamBehaviorStatus),
});

class TeammateRolesImpl : public TeammateRolesImplBase
{
  void execute(const TeammateRoles& p) override
  {
    theTeamBehaviorStatus.teammateRoles = p.teammateRoles;
    theLibCheck.inc(LibCheck::teammateRoles);
  }
};

MAKE_TEAM_SKILL_IMPLEMENTATION(TeammateRolesImpl);
