/**
 * @file CardDetails.cpp
 *
 * This file implements functions that are related to cards.
 *
 * @author Jesse Richter-Klug
 */

#include "CardDetails.h"

void CardCreatorBase::addToModuleInfo(CardCreatorBase* firstCreator, std::vector<ModuleBase::Info>& info)
{
  for(CardCreatorBase* cardCreator = firstCreator; cardCreator != nullptr; cardCreator = cardCreator->next)
  {
    auto requirements = cardCreator->getCardInfo().requires;
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

std::vector<const char*> CardCreatorBase::gatherSkillInfo(CardCreatorBase* firstCreator)
{
  std::vector<const char*> calls;
  for(CardCreatorBase* cardCreator = firstCreator; cardCreator != nullptr; cardCreator = cardCreator->next)
    for(const auto& call : cardCreator->getCardInfo().calls)
      calls.emplace_back(call);
  return calls;
}
