/**
 * @file ModuleManager.cpp
 * Implementation of a class representing the module manager.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ModuleManager.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include <algorithm>

ModuleManager::Configuration::RepresentationProvider::RepresentationProvider(const std::string& representation,
                                                                             const std::string& provider) :
  representation(representation), provider(provider)
{}

ModuleManager::ModuleManager(const std::set<ModuleBase::Category>& categories)
{
  for(ModuleBase* i = ModuleBase::first; i; i = i->next)
    if(categories.find(i->category) != categories.end())
      modules.push_back(ModuleState(i));
    else
      otherModules.push_back(ModuleState(i));
}

ModuleManager::~ModuleManager()
{
  destroy();
}

void ModuleManager::destroy()
{
  if(!providers.empty())
  {
    char buf[100];
    OutBinaryMemory out(buf);
    out << Configuration();
    InBinaryMemory in(buf);
    update(in, 0); // destruct everything
  }
}

const ModuleBase::Info* ModuleManager::find(const ModuleBase* module, const std::string& representation, bool all)
{
  for(const ModuleBase::Info* i = module->info; i->representation; ++i)
    if((i->update || all) && representation == i->representation)
      return i;
  return 0;
}

bool ModuleManager::calcShared(const Configuration& config)
{
  for(const auto& representationProvider : config.representationProviders)
  {
    auto module = std::find(modules.begin(), modules.end(), representationProvider.provider);
    auto otherModule = std::find(otherModules.begin(), otherModules.end(), representationProvider.provider);
    if(module == modules.end() && otherModule == otherModules.end())
    {
      // default can provide everthing that exists, but only that
      if(representationProvider.provider == "default")
      {
        bool exists = false;
        for(ModuleBase* i = ModuleBase::first; i && !exists; i = i->next)
          exists = find(i, representationProvider.representation, true) != 0;
        if(!exists)
        {
          OUTPUT_ERROR("Unknown representation " << representationProvider.representation << "!");
          return false;
        }
      }
      else
      {
        OUTPUT_ERROR("Module " << representationProvider.provider << " is unknown!");
        return false;
      }
    }
    else
    {
      bool provided = false;
      if(module != modules.end())
      {
        provided |= find(module->module, representationProvider.representation) != 0;
        if(!calcShared(config, representationProvider.representation, *module, modules, received, false))
          return false;
      }
      if(otherModule != otherModules.end())
      {
        provided |= find(otherModule->module, representationProvider.representation) != 0;
        if(!calcShared(config, representationProvider.representation, *otherModule, otherModules, sent, true))
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
                               const ModuleState& module, const std::list<ModuleState>& modules,
                               std::list<const char*>& received, bool silent)
{
  for(const auto& providerOfRepresentation : config.representationProviders)
    if(providerOfRepresentation.provider != module.module->name &&
       providerOfRepresentation.representation == representation &&
       (providerOfRepresentation.provider == "default" ||
        std::find(modules.begin(), modules.end(), providerOfRepresentation.provider) != modules.end()))
    {
      if(!silent)
        OUTPUT_ERROR(representation << " provided by more than one module!");
      return false;
    }

  for(const ModuleBase::Info* requirement = module.module->info; requirement->representation; ++requirement)
    if(!requirement->update)
    {
      bool provided = false;
      bool providedHere = false;
      for(const auto& providerOfRepresentation : config.representationProviders)
        if(providerOfRepresentation.representation == requirement->representation)
        {
          provided = true;
          if(providerOfRepresentation.provider == "default")
            providedHere = true;
          else
            for(const auto& module : modules)
              if(module == providerOfRepresentation.provider &&
                 find(module.module, providerOfRepresentation.representation) != 0)
                providedHere = true;
        }
      if(!provided)
      {
        if(!silent)
          OUTPUT_ERROR("No provider for required representation " << requirement->representation << " required by " << module.module->name);
        return false;
      }
      else if(!providedHere && std::find(received.begin(), received.end(), std::string(requirement->representation)) == received.end())
        received.push_back(requirement->representation);
    }
  return true;
}

void ModuleManager::update(In& stream, unsigned timeStamp)
{
  std::list<Provider> providersBackup(providers);
  std::list<const char*> sentBackup(sent),
                         receivedBackup(received);

  std::string representation,
              module;

  providers.clear();
  sent.clear();
  received.clear();

  // Remove all markings
  for(auto& m : modules)
  {
    m.requiredBackup = m.required;
    m.required = false;
  }

  stream >> config;

  // fill shared representations
  if(!calcShared(config))
  {
    rollBack(providersBackup, sentBackup, receivedBackup);
    return;
  }

  std::list<std::string> providedByDefault;
  for(const auto& rp : config.representationProviders)
  {
    if(rp.provider == "default")
      providedByDefault.push_back(rp.representation);
    else
      for(auto& m : modules)
        if(rp.provider == m.module->name)
        {
          for(const ModuleBase::Info* i = m.module->info; i->representation; ++i)
            if(i->update && rp.representation == i->representation)
            {
              providers.push_back(Provider(i->representation, &m, i->update));
              break;
            }
          m.required = true;
          break;
        }
  }

  if(!sortProviders(providedByDefault))
  {
    rollBack(providersBackup, sentBackup, receivedBackup);
    return;
  }

  // Delete all modules that are not required anymore
  for(auto& m : modules)
    if(!m.required && m.instance)
    {
      delete m.instance;
      m.instance = 0;
    }

  nextTimeStamp = timeStamp; // use this timestamp after execute was called
  this->timeStamp = 0; // invalid until execute was called
}

bool ModuleManager::sortProviders(const std::list<std::string>& providedByDefault)
{
  // The representations already provided. These are all that are received from the other process
  // and the ones that are provided by default.
  std::list<std::string> provided = providedByDefault;
  for(const char* r : received)
    provided.push_back(r);

  int remaining = static_cast<int>(providers.size()), // The number of entries not correct sequence so far.
      pushBackCount = remaining; // The number of push_backs still allowed. If zero, no valid sequence is possible.
  std::list<Provider>::iterator i = providers.begin();
  while(i != providers.end())
  {
    const ModuleBase::Info* j;
    for(j = i->moduleState->module->info; j->representation; ++j)
      if(!j->update && j->representation != i->representation && std::find(provided.begin(), provided.end(), j->representation) == provided.end())
        break;
    if(i->moduleState->module->info->representation && j->representation) // at least one requirement missing
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
        for(const auto& p : provided)
        {
          text += text == "" ? "" : ", ";
          text += p;
        }
        std::string text2 = "";
        while(i != providers.end())
        {
          text2 += text2 == "" ? "" : ", ";
          text2 += i->representation;
          ++i;
        }
        if(text == "")
          OUTPUT_ERROR("requirements missing for providers for " << text2 << ".");
        else
          OUTPUT_ERROR("only found consistent providers for " << text <<
                       ".\nRequirements missing for providers for " << text2 << ".");
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

void ModuleManager::rollBack(const std::list<Provider>& providers, const std::list<const char*>& sent,
                             const std::list<const char*>& received)
{
  this->providers = providers;
  this->sent = sent;
  this->received = received;
  for(auto& m : modules)
    m.required = m.requiredBackup;
}

void ModuleManager::load()
{
  InMapFile stream("modules.cfg");
  if(!stream.exists())
  {
    OUTPUT_ERROR("failed to load modules.cfg correctly.");
    ASSERT(true); // since when modules aren't loaded correctly ther come up other failures
  }
  update(stream, 0xffffffff);
}

void ModuleManager::execute()
{
  // Execute all providers in the given sequence
  for(auto& p : providers)
    if(p.moduleState->required)
    {
      if(!p.moduleState->instance)
        p.moduleState->instance = p.moduleState->module->createNew(); // returns 0 if provided by "default"
#ifdef TARGET_ROBOT
      unsigned timeStamp = Time::getCurrentSystemTime();
#endif
      if(p.moduleState->instance)
        p.update(*p.moduleState->instance);
#ifdef TARGET_ROBOT
      int duration = Time::getTimeSince(timeStamp);
      if(timeStamp > 20000 &&
         ((duration > 100 &&
           !Global::getDebugRequestTable().isActive("representation:JPEGImage") &&
           !Global::getDebugRequestTable().isActive("representation:Image")) ||
          duration > 500))
        TRACE("TIMING: providing %s took %d ms at %d s after start",
              p.representation, duration, timeStamp / 1000 - 10);
#endif
    }
  BH_TRACE;

  if(!timeStamp) // Configuration changed recently?
  {
    // all representations must be constructed now, so we can receive data
    timeStamp = nextTimeStamp;
    toSend.clear();
    for(const auto& s : sent)
      toSend.push_back(&Blackboard::getInstance()[s]);
    toReceive.clear();
    for(const auto& r : received)
      toReceive.push_back(&Blackboard::getInstance()[r]);
  }

  DEBUG_RESPONSE_ONCE("automated requests:ModuleTable")
  {
    Global::getDebugOut().bin << static_cast<unsigned>(modules.size());
    for(auto& m : modules)
    {
      Global::getDebugOut().bin << m.module->name << static_cast<unsigned char>(m.module->category);

      unsigned requirementsSize = 0;
      unsigned providersSize = 0;
      for(const ModuleBase::Info* j = m.module->info; j->representation; ++j)
        if(j->update)
          ++providersSize;
        else
          ++requirementsSize;

      Global::getDebugOut().bin << static_cast<unsigned>(requirementsSize);
      for(const ModuleBase::Info* j = m.module->info; j->representation; ++j)
        if(!j->update)
          Global::getDebugOut().bin << j->representation;

      Global::getDebugOut().bin << static_cast<unsigned>(providersSize);
      for(const ModuleBase::Info* j = m.module->info; j->representation; ++j)
        if(j->update)
          Global::getDebugOut().bin << j->representation;
    }

    Global::getDebugOut().bin << config;
    Global::getDebugOut().finishMessage(idModuleTable);
  }
}

void ModuleManager::readPackage(In& stream)
{
  unsigned timeStamp;
  stream >> timeStamp;
  // Communication is only possible if both sides are based on the same module request.
  if(timeStamp == this->timeStamp)
    for(Streamable* s : toReceive)
      stream >> *s;
  else
    stream.skip(10000000); // skip everything
}

void ModuleManager::writePackage(Out& stream) const
{
  stream << timeStamp;
  for(const Streamable* s : toSend)
    stream << *s;
}
