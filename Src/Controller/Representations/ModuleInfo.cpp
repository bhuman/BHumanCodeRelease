/**
 * @file Controller/Representations/ModuleInfo.cpp
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

bool ModuleInfo::handleMessage(InMessage& message)
{
  if(message.getMessageID() == idModuleTable)
  {
    int numOfModules;
    message.bin >> numOfModules;
    for(int i = 0; i < numOfModules; ++i)
    {
      Module module;
      int numOfRequirements;
      unsigned char category;
      message.bin >> module.name >> category >> numOfRequirements;
      module.category = static_cast<ModuleBase::Category>(category);
      module.requirements.resize(numOfRequirements);
      for(unsigned j = 0; j < module.requirements.size(); ++j)
      {
        message.bin >> module.requirements[j];
        representations.insert(module.requirements[j]);
      }
      int numOfRepresentations;
      message.bin >> numOfRepresentations;
      module.representations.resize(numOfRepresentations);
      for(unsigned j = 0; j < module.representations.size(); ++j)
      {
        message.bin >> module.representations[j];
        representations.insert(module.representations[j]);
      }
      std::list<Module>::iterator k;
      for(k = modules.begin(); k != modules.end() && *k < module; ++k)
        ;
      modules.insert(k, module);
    }
    message.bin >> config;
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
