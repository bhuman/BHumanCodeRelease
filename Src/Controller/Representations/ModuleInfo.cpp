/**
* @file Controller/Representations/ModuleInfo.cpp
*
* Implementation of class ModuleInfo
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "ModuleInfo.h"
#include "Tools/Module/ModuleManager.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <algorithm>

void ModuleInfo::clear()
{
  modules.clear();
  providers.clear();
}

bool ModuleInfo::handleMessage(InMessage& message, char processIdentifier)
{
  if(message.getMessageID() == idModuleTable)
  {
    int numOfModules;
    message.bin >> numOfModules;
    for(int i = 0; i < numOfModules; ++i)
    {
      Module module;
      module.processIdentifier = processIdentifier;
      int numOfRequirements;
      message.bin >> module.name >> module.category >> numOfRequirements;
      module.requirements.resize(numOfRequirements);
      for(unsigned j = 0; j < module.requirements.size(); ++j)
        message.bin >> module.requirements[j];
      int numOfRepresentations;
      message.bin >> numOfRepresentations;
      module.representations.resize(numOfRepresentations);
      for(unsigned j = 0; j < module.representations.size(); ++j)
      {
        message.bin >> module.representations[j];
        Provider provider;
        provider.representation = module.representations[j];
        provider.processIdentifier = processIdentifier;
        std::list<Provider>::iterator k = std::find(providers.begin(), providers.end(), provider);
        if(k == providers.end())
        {
          provider.modules.push_back(module.name);
          std::list<Provider>::iterator l;
          for(l = providers.begin(); l != providers.end() && *l < provider; ++l)
            ;
          providers.insert(l, provider);
        }
        else
          k->modules.push_back(module.name);
      }
      std::list<Module>::iterator k;
      for(k = modules.begin(); k != modules.end() && *k < module; ++k)
        ;
      modules.insert(k, module);
    }
    int numOfProviders;
    message.bin >> numOfProviders;
    for(int i = 0; i < numOfProviders; ++i)
    {
      Provider provider;
      provider.processIdentifier = processIdentifier;
      std::string module;
      message.bin >> provider.representation >> module;
      std::list<Provider>::iterator j = std::find(providers.begin(), providers.end(), provider);
      ASSERT(j != providers.end());
      j->selected = module;
    }
    timeStamp = SystemCall::getCurrentSystemTime();
    return true;
  }
  else
    return false;
}

std::string ModuleInfo::sendRequest(Out& stream, bool sort) const
{
  ModuleManager::Configuration config;

  for(std::list<Provider>::const_iterator i = providers.begin(); i != providers.end(); ++i)
  {
    if(i->selected != "")
    {
      std::list<Module>::const_iterator j = std::find(modules.begin(), modules.end(), i->selected);
      ASSERT(j != modules.end());
      for(std::vector<std::string>::const_iterator k = j->requirements.begin(); k != j->requirements.end(); ++k)
      {
        Provider provider;
        provider.representation = *k;
        provider.processIdentifier = i->processIdentifier;
        std::list<Provider>::const_iterator m;
        for(m = std::find(providers.begin(), providers.end(), provider);
            m != providers.end() && m->selected == "";
            m = std::find(++m, providers.end(), provider))
          ;
        if(m == providers.end())
        {
          // no provider in same process
          for(m = std::find(providers.begin(), providers.end(), *k);
              m != providers.end() && m->selected == "";
              m = std::find(++m, providers.end(), *k))
            ;
          if(m == providers.end())
            return "Error: no provider for required representation " + *k + " which is required by " + i->selected + " for providing " + i->representation;
        }
      }
    }
  }

  for(std::list<Provider>::const_iterator i = providers.begin(); i != providers.end(); ++i)
    if(i->selected != "")
      config.representationProviders.push_back(ModuleManager::Configuration::RepresentationProvider(i->representation, i->selected));

  if(sort)
    std::sort(config.representationProviders.begin(), config.representationProviders.end());

  stream << config;

  return "";
}
