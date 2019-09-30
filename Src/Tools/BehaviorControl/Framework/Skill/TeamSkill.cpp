/**
 * @file TeamSkill.cpp
 *
 * This file implements methods for team skills.
 *
 * @author Arne Hasselbring
 */

#include "TeamSkill.h"
#include "Platform/BHAssert.h"

thread_local TeamSkillRegistry* TeamSkillRegistry::theInstance = nullptr;

TeamSkillRegistry::TeamSkillRegistry(const char* skillConfig, ActivationGraph& activationGraph, TeamBehaviorStatus& teamBehaviorStatus) :
  SkillRegistryBase(activationGraph),
  theTeamBehaviorStatus(teamBehaviorStatus)
{
  ASSERT(!theInstance);
  theInstance = this;

  create(SkillImplementationCreatorList<TeamSkill>::first);
  resolveSkills(skillConfig);
}

TeamSkillRegistry::~TeamSkillRegistry()
{
  destroy();

  ASSERT(theInstance == this);
  theInstance = nullptr;
}
