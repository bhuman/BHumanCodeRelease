/**
 * @file Role.cpp
 *
 * This file implements the implementation of the Role skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamSkills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"

TEAM_SKILL_IMPLEMENTATION(RoleImpl,
{,
  IMPLEMENTS(Role),
  REQUIRES(LibCheck),
  MODIFIES(TeamBehaviorStatus),
});

class RoleImpl : public RoleImplBase
{
  void execute(const Role& p) override
  {
    theTeamBehaviorStatus.role = p.role;
    theLibCheck.inc(LibCheck::role);
  }
};

MAKE_TEAM_SKILL_IMPLEMENTATION(RoleImpl);
