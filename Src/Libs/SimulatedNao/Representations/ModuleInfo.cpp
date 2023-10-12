/**
 * @file Representations/ModuleInfo.cpp
 *
 * Implementation of class ModuleInfo
 *
 * @author Thomas Röfer
 */

#include "ModuleInfo.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include <algorithm>

void ModuleInfo::clear()
{
  modules.clear();
  representations.clear();
  for(Configuration::Thread& thread : config())
    thread.representationProviders.clear();
}

bool ModuleInfo::handleMessage(MessageQueue::Message message)
{
  if(message.id() == idModuleTable)
  {
    auto stream = message.bin();
    int numOfModules;
    stream >> numOfModules;
    for(int i = 0; i < numOfModules; ++i)
    {
      Module module;
      int numOfRequirements;
      stream >> module.name >> numOfRequirements;
      module.requirements.resize(numOfRequirements);
      for(unsigned j = 0; j < module.requirements.size(); ++j)
      {
        stream >> module.requirements[j];
        representations.insert(module.requirements[j]);
      }
      int numOfRepresentations;
      stream >> numOfRepresentations;
      module.representations.resize(numOfRepresentations);
      for(unsigned j = 0; j < module.representations.size(); ++j)
      {
        stream >> module.representations[j];
        representations.insert(module.representations[j]);
      }
      std::list<Module>::iterator k;
      for(k = modules.begin(); k != modules.end() && *k < module; ++k)
        ;
      modules.insert(k, module);
    }
    stream >> config;
    timestamp = Time::getCurrentSystemTime();
    return true;
  }
  else
    return false;
}

void ModuleInfo::sendRequest(Out& stream, bool sort)
{
  if(sort)
    for(Configuration::Thread& thread : config())
      std::sort(thread.representationProviders.begin(), thread.representationProviders.end());

  stream << config;
}
