/**
 * @file Skill.cpp
 *
 * This file implements methods for individual robot skills.
 *
 * @author Arne Hasselbring
 */

#include "Skill.h"
#include "Platform/BHAssert.h"

thread_local SkillRegistry* SkillRegistry::theInstance = nullptr;

SkillRegistry::SkillRegistry(const char* skillConfig, ActivationGraph& activationGraph, ArmMotionRequest& armMotionRequest,
                             BehaviorStatus& behaviorStatus, HeadMotionRequest& headMotionRequest, MotionRequest& motionRequest, TeamTalk& teamTalk) :
  SkillRegistryBase(activationGraph),
  theArmMotionRequest(armMotionRequest),
  theBehaviorStatus(behaviorStatus),
  theHeadMotionRequest(headMotionRequest),
  theMotionRequest(motionRequest),
  theTeamTalk(teamTalk)
{
  ASSERT(!theInstance);
  theInstance = this;

  create(SkillImplementationCreatorList<Skill>::first);
  resolveSkills(skillConfig);
}

SkillRegistry::~SkillRegistry()
{
  destroy();

  ASSERT(theInstance == this);
  theInstance = nullptr;
}
