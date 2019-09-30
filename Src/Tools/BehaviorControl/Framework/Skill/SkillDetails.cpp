/**
 * @file SkillDetails.cpp
 *
 * This file implements functions that are related to skills.
 *
 * @author Arne Hasselbring
 */

#include "SkillDetails.h"

void SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorBase* firstCreator, std::vector<ModuleBase::Info>& info)
{
  for(SkillImplementationCreatorBase* skill = firstCreator; skill != nullptr; skill = skill->next)
  {
    auto requirements = skill->getSkillInfo().requires;
    for(const char* requirement : requirements)
    {
      bool found = false;
      for(auto& i : info)
        if(!i.update && (found = (std::string(requirement) == i.representation)))
          break;
      if(!found)
        info.emplace_back(requirement, nullptr);
    }
  }
}
