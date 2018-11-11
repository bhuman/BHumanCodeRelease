/**
 * @file BehaviorOptionRegistry.cpp
 *
 * This file implements a class to create and store instances of behavior options.
 *
 * @author Arne Hasselbring
 */

#include "BehaviorOptionRegistry.h"

thread_local BehaviorOptionRegistry* BehaviorOptionRegistry::theInstance = nullptr;

BehaviorOptionRegistry::BehaviorOptionRegistry(ActivationGraph* activationGraph) :
  theActivationGraph(activationGraph)
{
  ASSERT(theInstance == nullptr);
  theInstance = this;

  for(BehaviorOptionBase* bob = BehaviorOptionBase::first; bob != nullptr; bob = bob->next)
    behaviorOptions.emplace_back(bob);
}

BehaviorOptionRegistry::~BehaviorOptionRegistry()
{
  for(auto& option : behaviorOptions)
    delete option.instance;

  ASSERT(theInstance == this);
  theInstance = nullptr;
}

void BehaviorOptionRegistry::modifyAllParameters()
{
  for(auto& option : behaviorOptions)
    if(option.instance != nullptr)
      option.instance->modifyParameters();
}

void BehaviorOptionRegistry::preProcess()
{
  for(auto& option : behaviorOptions)
    if(option.instance != nullptr)
      option.instance->preProcess();
}

void BehaviorOptionRegistry::postProcess()
{
  for(auto& option : behaviorOptions)
    if(option.instance != nullptr)
      option.instance->postProcess();
}
