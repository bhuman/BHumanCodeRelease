/**
 * @file TeamBehaviorControl.cpp
 *
 * This file implements the module that executes the team behavior.
 *
 * @author Arne Hasselbring
 */

#include "TeamBehaviorControl.h"

MAKE_MODULE(TeamBehaviorControl, behaviorControl, TeamBehaviorControl::getExtModuleInfo);

TeamBehaviorControl::TeamBehaviorControl() :
  theTeamSkillRegistry("teamSkills.cfg", const_cast<TeamActivationGraph&>(theTeamActivationGraph), theTeamBehaviorStatus),
  theTeamCardRegistry(const_cast<TeamActivationGraph&>(theTeamActivationGraph))
{
  theTeamSkillRegistry.checkSkills(CardCreatorBase::gatherSkillInfo(CardCreatorList<TeamCard>::first));
}

std::vector<ModuleBase::Info> TeamBehaviorControl::getExtModuleInfo()
{
  auto result = TeamBehaviorControl::getModuleInfo();

  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<TeamSkill>::first, result);
  CardCreatorBase::addToModuleInfo(CardCreatorList<TeamCard>::first, result);

  return result;
}

void TeamBehaviorControl::update(TeamActivationGraph& teamActivationGraph)
{
  teamActivationGraph.graph.clear();
  teamActivationGraph.currentDepth = 0;

  theTeamSkillRegistry.modifyAllParameters();
  theTeamCardRegistry.modifyAllParameters();

  theTeamSkillRegistry.preProcess(theFrameInfo.time);
  theTeamCardRegistry.preProcess(theFrameInfo.time);

  teamActivationGraph.graph.emplace_back("TeamBehaviorControl", 0, "", theFrameInfo.time, 0, std::vector<std::string>());
  CardBase* card = theTeamCardRegistry.getCard(rootCard);
  ASSERT(card);
  card->call();

  theTeamCardRegistry.postProcess();
  theTeamSkillRegistry.postProcess();

  theLibCheck.performTeamCheck();
}
