/**
 * @file BehaviorControl.cpp
 *
 * This file implements a module that describes the robot behavior.
 *
 * @author Arne Hasselbring
 */

#include "BehaviorControl.h"

#include "Tools/BehaviorOption.h"

MAKE_MODULE_WITH_INFO(BehaviorControl, behaviorControl, BehaviorControl::getExtModuleInfo);

std::vector<ModuleBase::Info> BehaviorControl::getExtModuleInfo()
{
  auto result = BehaviorControlBase::getModuleInfo();
  BehaviorOptionBase::addToModuleInfo(result);
  return result;
}

BehaviorControl::BehaviorControl() :
  registry(const_cast<ActivationGraph*>(&theActivationGraph)),
  soccerBehavior(*registry.getOption<BehaviorOptionInterface>("Soccer")),
  headControl2018(*registry.getOption<BehaviorOptionInterface>("HeadControl2018"))
{
}

void BehaviorControl::update(ActivationGraph& activationGraph)
{
  registry.modifyAllParameters();

  registry.theBehaviorStatus.walkingTo = theRobotPose.translation;
  registry.theBehaviorStatus.shootingTo = theRobotPose.translation;

  activationGraph.graph.clear();
  activationGraph.currentDepth = 0;

  registry.preProcess();

  soccerBehavior.execute();
  headControl2018.execute();

  registry.postProcess();
}
