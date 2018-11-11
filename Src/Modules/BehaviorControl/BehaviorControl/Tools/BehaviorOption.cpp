/**
 * @file BehaviorOption.cpp
 *
 * This file defines functions and symbols related to behavior options.
 *
 * @author Arne Hasselbring
 */

#include "BehaviorOption.h"

#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"

#include <cctype>

BehaviorOptionBase* BehaviorOptionBase::first = nullptr;

void loadBehaviorOptionParameters(Streamable& parameters, const char* behaviorOptionName, const char* fileName)
{
  std::string name;
  if(!fileName)
  {
    name = behaviorOptionName;
    name[0] = static_cast<char>(tolower(name[0]));
    if(name.size() > 1 && isupper(name[1]))
      for(int i = 1; i + 1 < static_cast<int>(name.size()) && isupper(name[i + 1]); ++i)
        name[i] = static_cast<char>(tolower(name[i]));
    name += ".cfg";
  }
  else
    name = fileName;
  InMapFile stream("BehaviorControl/" + name);
  ASSERT(stream.exists());
  stream >> parameters;
}

void BehaviorOptionBase::addToModuleInfo(std::vector<ModuleBase::Info>& info)
{
  for(BehaviorOptionBase* bob = BehaviorOptionBase::first; bob != nullptr; bob = bob->next)
  {
    auto requirements = bob->getRequirements();
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
