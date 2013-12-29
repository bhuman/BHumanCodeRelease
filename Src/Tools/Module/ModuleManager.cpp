/**
 * @file ModuleManager.cpp
 * Implementation of a class representing the module manager.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ModuleManager.h"
#include "Platform/BHAssert.h"
#include "Tools/Streams/InStreams.h"
#include <algorithm>
#include <set>

ModuleManager::Configuration::RepresentationProvider::RepresentationProvider(const std::string& representation,
                                                                             const std::string& provider)
: representation(representation),
  provider(provider) {}

DefaultModule::DefaultModule() :
  ModuleBase("default", "Infrastructure")
{
  // base class will add this module to the list of all modules, but it should not
  // so remove it again
  first = first->next;
}

void DefaultModule::setRepresentations(const std::list<ModuleManager::ModuleState>& modules)
{
  for(std::list<ModuleManager::ModuleState>::const_iterator i = modules.begin(); i != modules.end(); ++i)
    for(Representations::List::const_iterator j = i->module->representations.begin(); j != i->module->representations.end(); ++j)
      if(std::find(representations.begin(), representations.end(), j->name) == representations.end())
        representations.push_back(*j);
}

ModuleManager::ModuleManager(const char** categories, size_t numOfCategories) :
  timeStamp(0),
  defaultModule(new DefaultModule),
  otherDefaultModule(new DefaultModule)
{
  std::set<std::string> filter;
  for(int i = 0; i < (int) numOfCategories; ++i)
    filter.insert(categories[i]);

  for(ModuleBase* i = ModuleBase::first; i; i = i->next)
    if(filter.find(i->category) != filter.end())
      modules.push_back(ModuleState(i));
    else
      otherModules.push_back(ModuleState(i));

  defaultModule->setRepresentations(modules);
  modules.push_back(ModuleState(defaultModule));

  otherDefaultModule->setRepresentations(otherModules);
  otherModules.push_back(ModuleState(otherDefaultModule));
}

ModuleManager::~ModuleManager()
{
  destroy();
}

void ModuleManager::destroy()
{
  if(defaultModule) // prevent multiple destructions
  {
    char buf[100];
    OutBinaryMemory out(buf);
    out << Configuration();
    InBinaryMemory in(buf);
    update(in); // destruct everything
    delete defaultModule;
    defaultModule = 0;
    delete otherDefaultModule;
    otherDefaultModule = 0;
  }
}

bool ModuleManager::calcShared(const Configuration& config)
{
  for(const auto& representationProvider : config.representationProviders)
  {
    auto module = std::find(modules.begin(), modules.end(), representationProvider.provider);
    auto otherModule = std::find(otherModules.begin(), otherModules.end(), representationProvider.provider);
    if(module == modules.end() && otherModule == otherModules.end())
    {
      OUTPUT_ERROR("Module " << representationProvider.provider << " is unknown!");
      return false;
    }
    else
    {
      bool provided = false;
      if(module != modules.end())
      {
        const auto& representations = module->module->representations;
        provided |= std::find(representations.begin(), representations.end(), representationProvider.representation) != representations.end();
        if(!calcShared(config, representationProvider.representation, *module, modules))
          return false;
      }
      if(otherModule != otherModules.end())
      {
        const auto& representations = otherModule->module->representations;
        provided |= std::find(representations.begin(), representations.end(), representationProvider.representation) != representations.end();
        if(!calcShared(config, representationProvider.representation, *otherModule, otherModules))
          return false;
      }
      if(!provided)
      {
        OUTPUT_ERROR(representationProvider.provider << " does not provide " << representationProvider.representation << "!");
        return false;
      }
    }
  }
  return true;
}

bool ModuleManager::calcShared(const Configuration& config, const std::string& representation,
                               const ModuleState& module, const std::list<ModuleState>& modules)
{
  for(const auto& providerOfRepresentation : config.representationProviders)
    if(providerOfRepresentation.provider != module.module->name &&
       providerOfRepresentation.representation == representation &&
       std::find(modules.begin(), modules.end(), providerOfRepresentation.provider) != modules.end())
    {
      OUTPUT_ERROR(representation << " provided by more than one module!");
      return false;
    }

  for(const auto& requirement : module.module->requirements)
  {
    bool provided = false;
    bool providedHere = false;
    for(const auto& providerOfRepresentation : config.representationProviders)
      if(providerOfRepresentation.representation == requirement.name)
      {
        provided = true;
        for(const auto& module : modules)
          if(module == providerOfRepresentation.provider &&
             std::find(module.module->representations.begin(), module.module->representations.end(),
                       providerOfRepresentation.representation) != module.module->representations.end())
          providedHere = true;
      }
    if(!provided)
    {
      OUTPUT_ERROR("No provider for required representation " << requirement.name << "!");
      return false;
    }
    else if(!providedHere && std::find(shared.begin(), shared.end(), std::string(requirement.name)) == shared.end())
      shared.push_back(std::string(requirement.name));
  }
  return true;
}

void ModuleManager::update(In& stream, unsigned timeStamp)
{
  std::list<Provider> providersToDelete(providers),
                      providersToCreate,
                      providersBackup(providers);
  std::list<Shared> sharedToDelete(shared),
                    sharedToCreate,
                    sharedBackup(shared);

  std::string representation,
              module;

  providers.clear();
  selected.clear();
  shared.clear();

  // Remove all markings
  for(std::list<ModuleState>::iterator j = modules.begin(); j != modules.end(); ++j)
  {
    j->requiredBackup = j->required;
    j->required = false;
  }

  Configuration config;
  stream >> config;

  // fill shared representations
  if(!calcShared(config))
  {
    rollBack(providersBackup, sharedBackup);
    return;
  }

  for(const auto& rp : config.representationProviders)
  {
    std::list<ModuleState>::iterator j;
    for(j = modules.begin(); j != modules.end(); ++j)
      if(rp.provider == j->module->name)
      {
        Representations::List::const_iterator i;
        for(i = j->module->representations.begin(); i != j->module->representations.end(); ++i)
          if(rp.representation == i->name)
          {
            Provider provider(i->name, &*j, i->update, i->create, i->free);
            std::list<Provider>::iterator m = std::find(providers.begin(), providers.end(), provider);
            if(m == providers.end())
            {
              selected[i->name] = j->module->name;
              providers.push_back(provider);
              // Is the representation provided one of the shared ones?
              std::list<Shared>::iterator k = std::find(shared.begin(), shared.end(), rp.representation);
              if(k != shared.end()) // yes
              {
                k->out = i->out; // set write handler
                k->in = 0; // remove read handler
              }
              // This provider may have a requirement that is a shared representation
              for(k = shared.begin(); k != shared.end(); ++k)
                if(!k->out && !k->in) // only search those with unknown state
                {
                  Requirements::List::const_iterator m = std::find(j->module->requirements.begin(), j->module->requirements.end(), k->representation);
                  if(m != j->module->requirements.end()) // yes, one of the requirements is shared
                  {
                    k->create = m->create; // so we know how to create, free, and read it
                    k->free = m->free;
                    k->in = m->in;
                    // Is the representation already constructed
                    std::list<Shared>::iterator n = std::find(sharedToDelete.begin(), sharedToDelete.end(), k->representation);
                    if(n == sharedToDelete.end()) // no, representation is new
                      sharedToCreate.push_back(*k);
                    else // it already exists and is still needed, so don't delete it
                      sharedToDelete.erase(n);
                  }
                }
              // Is the representation already constructed
              m = std::find(providersToDelete.begin(), providersToDelete.end(), provider);
              if(m == providersToDelete.end()) // no, representation is new
                providersToCreate.push_back(providers.back());
              else // it already exists and is still needed, so don't delete it
                providersToDelete.erase(m);
            }
            break;
          }
        j->required = true;
        break;
      }
  }

  if(!sortProviders())
  {
    rollBack(providersBackup, sharedBackup);
    return;
  }

  // Delete all modules that are not required anymore
  for(std::list<ModuleState>::iterator j = modules.begin(); j != modules.end(); ++j)
    if(!j->required && j->instance)
    {
      delete j->instance;
      j->instance = 0;
    }

  // Delete all representations that are not required anymore
  for(std::list<Provider>::iterator j = providersToDelete.begin(); j != providersToDelete.end(); ++j)
  {
    // don't delete representations that are shared now
    std::list<Shared>::iterator k = std::find(shared.begin(), shared.end(), j->representation);
    if(k == shared.end() || !k->in)
      j->free();
  }

  // Delete all shared representations that are not required anymore
  for(std::list<Shared>::iterator j = sharedToDelete.begin(); j != sharedToDelete.end(); ++j)
    if(j->in) // don't delete representations that would immediately be recreated by a provider
      if(std::find(providersToCreate.begin(), providersToCreate.end(), j->representation) == providersToCreate.end())
        j->free();

  // Create all shared representations that are missing
  for(std::list<Shared>::iterator j = sharedToCreate.begin(); j != sharedToCreate.end(); ++j)
    if(j->in) // don't create representations that were provided by a provider before
      if(std::find(providersToDelete.begin(), providersToDelete.end(), j->representation) == providersToDelete.end())
        j->create();

  // Create all representations that are missing
  for(std::list<Provider>::iterator j = providersToCreate.begin(); j != providersToCreate.end(); ++j)
  {
    // don't create representations that were shared before
    std::list<Shared>::iterator k = std::find(sharedToDelete.begin(), sharedToDelete.end(), j->representation);
    if(k == sharedToDelete.end() || !k->in)
      j->create();
  }

  this->timeStamp = timeStamp;
}

bool ModuleManager::sortProviders()
{
  std::list<std::string> provided; /**< The representation already provided. */

  // Add shared representations that are received by this process
  for(std::list<Shared>::const_iterator i = shared.begin(); i != shared.end(); ++i)
    if(i->in)
      provided.push_back(i->representation);

  int remaining = providers.size(), /**< The number of entries not correct sequence so far. */
      pushBackCount = remaining; /**< The number of push_backs still allowed. If zero, no valid sequence is possible. */
  std::list<Provider>::iterator i = providers.begin();
  while(i != providers.end())
  {
    const Requirements::List& requirements = i->moduleState->module->requirements;
    Requirements::List::const_iterator j;
    for(j = requirements.begin(); j != requirements.end(); ++j)
      if(j->name != i->representation && std::find(provided.begin(), provided.end(), j->name) == provided.end())
        break;
    if(requirements.begin() != requirements.end() && j != requirements.end()) // at least one requirement missing
    {
      if(pushBackCount) // still one left to try
      {
        providers.push_back(*i);
        i = providers.erase(i);
        --pushBackCount;
      }
      else // we checked all, none was satisfied
      {
        std::string text;
        for(std::list<std::string>::const_iterator k = provided.begin(); k != provided.end(); ++k)
        {
          text += text == "" ? "" : ", ";
          text += *k;
        }
        std::string text2 = "";
        while(i != providers.end())
        {
          text2 += text2 == "" ? "" : ", ";
          text2 += i->representation;
          ++i;
        }
        if(text == "")
        {
          OUTPUT_ERROR("requirements missing for providers for " << text2 << ".");
        }
        else
        {
          OUTPUT_ERROR("only found consistent providers for " << text <<
                       ".\nRequirements missing for providers for " << text2 << ".");
        }
        return false;
      }
    }
    else // we found one with all requirements fulfilled
    {
      provided.push_back(i->representation); // add representation provided
      ++i; // continue with next provider
      --remaining; // we have one less to go,
      pushBackCount = remaining; // and the search starts again
    }
  }
  return true;
}

void ModuleManager::rollBack(const std::list<Provider>& providers, const std::list<Shared>& shared)
{
  this->providers = providers;
  this->shared = shared;
  for(std::list<ModuleState>::iterator j = modules.begin(); j != modules.end(); ++j)
    j->required = j->requiredBackup;
}

void ModuleManager::load()
{
  InMapFile stream("modules.cfg");
  if(!stream.exists())
  {
    OUTPUT_ERROR("failed to load modules.cfg correctly.");
    ASSERT(true); // since when modules aren't loaded correctly ther come up other failures
  }
  update(stream);
}

void ModuleManager::execute()
{
  // Execute all providers in the given sequence
  for(std::list<Provider>::iterator i = providers.begin(); i != providers.end(); ++i)
    if(i->moduleState->required)
    {
      if(!i->moduleState->instance)
        i->moduleState->instance = i->moduleState->module->createNew(); // returns 0 if provided by "default"
#ifdef TARGET_ROBOT
      unsigned timeStamp = SystemCall::getCurrentSystemTime();
#endif
      if(i->moduleState->instance)
        i->update(*i->moduleState->instance);
#ifdef TARGET_ROBOT
      int duration = SystemCall::getTimeSince(timeStamp);
      if(duration > 100 &&
         ((!Global::getDebugRequestTable().isActive("representation:JPEGImage") &&
           !Global::getDebugRequestTable().isActive("representation:Image")) ||
          duration > 500))
        TRACE("TIMING: providing %s took %d ms at %d s after start",
              i->representation.c_str(), duration, timeStamp / 1000 - 10);
#endif
    }
  BH_TRACE;

  DEBUG_RESPONSE_ONCE("automated requests:ModuleTable",
  {
    Global::getDebugOut().bin << (unsigned) modules.size();
    for(std::list<ModuleState>::const_iterator i = modules.begin(); i != modules.end(); ++i)
    {
      Global::getDebugOut().bin << i->module->name << i->module->category;
      const Requirements::List& requirements = i->module->requirements;
      Global::getDebugOut().bin << (unsigned) requirements.size();
      for(Requirements::List::const_iterator j = requirements.begin(); j != requirements.end(); ++j)
        Global::getDebugOut().bin << j->name;
      const Representations::List& representations = i->module->representations;
      Global::getDebugOut().bin << (unsigned) representations.size();
      for(Representations::List::const_iterator j = representations.begin(); j != representations.end(); ++j)
        Global::getDebugOut().bin << j->name;
    }
    Global::getDebugOut().bin << (unsigned) selected.size();
    for(std::map<const char*, const char*>::const_iterator i = selected.begin(); i != selected.end(); ++i)
      Global::getDebugOut().bin << i->first << i->second;
    Global::getDebugOut().finishMessage(idModuleTable);
  });
}

void ModuleManager::readPackage(In& stream)
{
  unsigned timeStamp;
  stream >> timeStamp;
  // Communication is only possible if both sides are based on the same module request.
  if(timeStamp == this->timeStamp)
  {
    for(std::list<Shared>::const_iterator i = shared.begin(); i != shared.end(); ++i)
      if(i->in)
        i->in(stream);
  }
  else
    stream.skip(10000000); // skip everything
}

void ModuleManager::writePackage(Out& stream) const
{
  stream << timeStamp;
  for(std::list<Shared>::const_iterator i = shared.begin(); i != shared.end(); ++i)
    if(i->out)
      i->out(stream);
}

std::vector<std::string> ModuleManager::getCurrentRepresentatioNames() const
{
  std::vector<std::string> data;
  for(const Provider& provider : providers)
  {
    data.push_back(provider.representation);
  }
  return data;
}
