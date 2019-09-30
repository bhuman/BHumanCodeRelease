/**
 * @file SkillRegistryBase.cpp
 *
 * This file implements a class that manages skill interfaces and implementations.
 *
 * @author Arne Hasselbring
 */

#include "SkillRegistryBase.h"
#include "SkillDetails.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"

SkillRegistryBase::SkillRegistryBase(ActivationGraph& activationGraph) :
  theActivationGraph(activationGraph)
{
}

SkillRegistryBase::~SkillRegistryBase()
{
  ASSERT(skillImplementations.empty());
  ASSERT(skillInterfaces.empty());
}

void SkillRegistryBase::create(SkillImplementationCreatorBase* firstCreator)
{
  for(SkillImplementationCreatorBase* skill = firstCreator; skill != nullptr; skill = skill->next)
    skillImplementations.emplace_back(skill);
}

void SkillRegistryBase::destroy()
{
  for(SkillImplementationState& skillImplementation : skillImplementations)
    delete skillImplementation.instance;
  skillImplementations.clear();
  for(SkillInterfaceState& skillInterface : skillInterfaces)
    delete skillInterface.instance;
  skillInterfaces.clear();
}

void SkillRegistryBase::modifyAllParameters()
{
  for(SkillImplementationState& skillImplementation : skillImplementations)
    if(skillImplementation.instance)
      skillImplementation.instance->modifyParameters();
}

void SkillRegistryBase::preProcess(unsigned frameTime)
{
  currentFrameTime = frameTime;
  for(SkillImplementationState& skillImplementation : skillImplementations)
    if(skillImplementation.instance)
      skillImplementation.instance->preProcess();
  for(SkillInterfaceState& skillInterface : skillInterfaces)
    skillInterface.instance->preProcess();
}

void SkillRegistryBase::postProcess()
{
  for(SkillInterfaceState& skillInterface : skillInterfaces)
    skillInterface.instance->postProcess();
  for(SkillImplementationState& skillImplementation : skillImplementations)
    if(skillImplementation.instance)
      skillImplementation.instance->postProcess();
  lastFrameTime = currentFrameTime;
}

void SkillRegistryBase::resolveSkills(const char* skillConfig)
{
  // Instantiate all skill interfaces for which at least one implementation exists.
  for(SkillImplementationState& skillImplementation : skillImplementations)
  {
    for(SkillInterfaceCreator& impl : skillImplementation.skill->getSkillInfo().implements)
    {
      bool found = false;
      for(SkillInterfaceState& skillInterface : skillInterfaces)
        if(skillInterface.name == impl.name)
        {
          found = true;
          break;
        }
      if(!found)
        skillInterfaces.emplace_back(impl);
    }
  }

#ifndef NDEBUG
  // Check if skills are referenced that do not exist.
  for(SkillImplementationState& skillImplementation : skillImplementations)
  {
    for(const char* call : skillImplementation.skill->getSkillInfo().calls)
      if(!getSkillInterface(call))
        FAIL("The skill implementation " << skillImplementation.skill->name << " calls a skill that is not implemented: " << call);
  }
#endif

  // Set the implementation for all skill interfaces.
  InMapFile stream(std::string("BehaviorControl/") + skillConfig);
  ASSERT(stream.exists());
  stream >> config;
  // For each skill interface: Look for all implementations that implement that interface.
  for(SkillInterfaceState& skillInterface : skillInterfaces)
  {
    std::vector<SkillImplementationState*> possibleImplementations;
    for(SkillImplementationState& skillImplementation : skillImplementations)
    {
      for(SkillInterfaceCreator& impl : skillImplementation.skill->getSkillInfo().implements)
      {
        if(skillInterface.name == impl.name)
          possibleImplementations.push_back(&skillImplementation);
      }
    }
    ASSERT(!possibleImplementations.empty());

    // Find an assignment in the configuration file.
    Configuration::SkillImplementation* configuredAssignment = nullptr;
    for(Configuration::SkillImplementation& skillImplementation : config.skillImplementations)
    {
      if(skillInterface.name == skillImplementation.skill)
      {
#ifndef NDEBUG
        if(configuredAssignment)
          FAIL("The skill " << skillInterface.name << " is assigned multiple times in " << skillConfig << ".");
#endif
        configuredAssignment = &skillImplementation;
#ifdef NDEBUG
        break;
#endif
      }
    }

    if(possibleImplementations.size() == 1)
    {
      if(configuredAssignment)
        FAIL("Unambiguous skill assignments must not appear in " << skillConfig << ".");
      if(!possibleImplementations.front()->instance)
        possibleImplementations.front()->instance = possibleImplementations.front()->skill->createNew();
      skillInterface.instance->setImplementation(possibleImplementations.front()->instance);
    }
    else
    {
      if(!configuredAssignment)
        FAIL("The skill " << skillInterface.name << " has multiple implementations but is not assigned in " << skillConfig << ".");

      bool implFound = false;
      for(SkillImplementationState* skillImplementation : possibleImplementations)
      {
        if(configuredAssignment->implementation == skillImplementation->skill->name)
        {
          if(!skillImplementation->instance)
            skillImplementation->instance = skillImplementation->skill->createNew();
          skillInterface.instance->setImplementation(skillImplementation->instance);
          implFound = true;
          break;
        }
      }
      if(!implFound)
        FAIL("The skill " << skillInterface.name << " is assigned an implementation called " << configuredAssignment->implementation << " which does not exist.");
    }
  }
}

void SkillRegistryBase::checkSkills(const std::vector<const char*>& skillNames)
{
#ifndef NDEBUG
  for(const char* skill : skillNames)
    if(!getSkillInterface(skill))
      FAIL("The skill " << skill << " is called but not implemented.");
#endif
}

SkillInterface* SkillRegistryBase::getSkillInterface(const std::string& skillName)
{
  for(SkillInterfaceState& skillInterface : skillInterfaces)
  {
    if(skillName == skillInterface.name)
      return skillInterface.instance;
  }
  return nullptr;
}

SkillRegistryBase::SkillImplementationState::SkillImplementationState(SkillImplementationCreatorBase* skill) :
  skill(skill)
{}

SkillRegistryBase::SkillInterfaceState::SkillInterfaceState(const SkillInterfaceCreator& skill) :
  name(skill.name),
  instance(skill.createNew())
{}
