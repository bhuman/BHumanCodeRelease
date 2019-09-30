/**
 * @file Tools/Module/ModuleGraphCreator.cpp
 *
 * Implementation of a class representing the module graph creator.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#include "ModuleGraphCreator.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"

#include <algorithm>
#include <map>

ModuleGraphCreator::ModuleGraphCreator(const Configuration& config)
  : required(config().size()), received(config().size()), sent(config().size()), providers(config().size()), typeInfo(true)
{
  for(auto& r : received)
    r.resize(config().size());
  for(auto& s : sent)
    s.resize(config().size());

  for(ModuleBase* i = ModuleBase::first; i; i = i->next)
    modules.emplace(i->name, i);

  for(auto& r : required)
    r.resize(modules.size());
}

std::vector<ModuleBase::Info>::const_iterator ModuleGraphCreator::find(const std::vector<ModuleBase::Info>& info, const std::string& representation, bool required)
{
  for(auto i = info.cbegin(); i != info.cend(); ++i)
    if((!i->update == required) && representation == i->representation)
      return i;
  return info.cend();
}

bool ModuleGraphCreator::calcShared(const Configuration& config)
{
  if(config().empty() && !config.defaultRepresentations.empty())
  {
    OUTPUT_ERROR("Default representation " << config.defaultRepresentations.front() << " is not required anywhere!");
    return false;
  }
  else if(config().empty())
    return true;

  // Calculate all threads.
  for(std::size_t index = 0; index < config().size(); index++)
  {
    // Check all providers and calculate shared.
    for(const auto& rp : config()[index].representationProviders)
    {
      // Does module exist?
      if(modules.find(rp.provider) == modules.end())
      {
        OUTPUT_ERROR(config()[index].name << ": Module " << rp.provider << " is unknown!");
        return false;
      }

      auto module = modules.find(rp.provider);
      ASSERT(module != modules.end());

      // Can module provide this representation?
      const std::vector<ModuleBase::Info>& info = module->second->getModuleInfo();
      if(find(info, rp.representation) == info.cend())
      {
        OUTPUT_ERROR(config()[index].name << ": " << rp.provider << " does not provide " << rp.representation << "!");
        return false;
      }

      // Calculate shared
      if(!calcShared(config, index, rp.representation, module->second, received[index]))
        return false;
    }

    // Set the senders to the receivers.
    for(std::size_t thread = 0; thread < config().size(); ++thread)
      if(thread != index)
      {
        sent[thread][index] = received[index][thread];

        // Modify sent representations if alias.
        // Example: receive UpperBall -> sent Ball
        for(std::size_t i = 0; i < sent[thread][index].size(); i++)
        {
          // Check whether the sent representation is provided.
          bool provided = false;
          for(const auto& rp : config()[thread].representationProviders)
          {
            if(sent[thread][index][i] == rp.representation)
            {
              if(rp.provider == "ConfigurationDataProvider")
                OUTPUT_WARNING("Representation " << rp.representation
                               << " should be provided by module ConfigurationDataProvider in thread "
                               << config()[index].name);
              provided = true;
            }
          }
          if(provided)
            continue;

          // Representation is alias, find right representation.
          for(const auto& rp : config()[thread].representationProviders)
          {
            const std::string& repr(sent[thread][index][i]);
            // Find right representation.
            if(repr.substr(config()[thread].name.size(), repr.size()) == rp.representation)
            {
              auto module = modules.find(rp.provider);
              ASSERT(module != modules.end());
              for(const ModuleBase::Info& requirement : module->second->getModuleInfo())
              {
                // Set right representation
                if(requirement.update && repr.substr(config()[thread].name.size(), repr.size()) == requirement.representation)
                {
                  sent[thread][index][i] = requirement.representation;
                  provided = true;
                  break;
                }
              }
              break;
            }
          }
          // If that's false, there's something wrong here.
          ASSERT(provided);
        }
      }
  }

  // Default can provide everything that exists, but only that.
  for(const std::string& representation : config.defaultRepresentations)
  {
    for(const auto& m : modules)
    {
      const std::vector<ModuleBase::Info>& info = m.second->getModuleInfo();
      if(find(info, representation, true) != info.cend())
        goto found;
      // Also check alias
      for(const Configuration::Thread& con : config())
        if(find(info, con.name + representation, true) != info.cend())
          goto found;
    }
    OUTPUT_ERROR("Default representation " << representation << " is not required anywhere!");
    return false;
  found:
    ;
  }

  return true;
}

bool ModuleGraphCreator::calcShared(const Configuration& config, std::size_t index,
                                    const std::string& representation, const ModuleBase* module,
                                    std::vector<std::vector<const char*>>& received) const
{
  // Checks if the representation is not also provided by default.
  if(std::find(config.defaultRepresentations.begin(), config.defaultRepresentations.end(), representation) != config.defaultRepresentations.end())
  {
    OUTPUT_ERROR(config()[index].name << ": " << representation << " is also provided by default!");
    return false;
  }

  // Check if all requirements are provided by this or another thread
  for(const ModuleBase::Info& requirement : module->getModuleInfo())
  {
    if(!requirement.update)
    {
      bool provided = false;

      // Check if provided by default.
      for(const std::string& representation : config.defaultRepresentations)
      {
        if((provided = requirement.representation == representation))
          break;
        // Check if an alias is provided by default.
        for(const Configuration::Thread& thread : config())
          if((provided = requirement.representation == thread.name + representation))
            break;
        if(provided)
          break;
      }
      if(provided)
        continue;

      bool maybeAlias = false;
      int aliasThreadIndex = -1;

      // Check whether a representation may be expected from a specific thread.
      for(size_t thread = 0; thread < config().size(); ++thread)
        if(thread != index)
        {
          if(config()[thread].name.size() < std::string(requirement.representation).size() &&
             std::mismatch(config()[thread].name.begin(), config()[thread].name.end(),  std::string(requirement.representation).begin()).first == config()[thread].name.end())
          {
            maybeAlias = true;
            aliasThreadIndex = static_cast<int>(thread);
            break;
          }
        }

      // Check if required representation is provided in own thread.
      for(const auto& rp : config()[index].representationProviders)
        if((provided = rp.representation == requirement.representation))
          break;

      if(!provided)
      {
        // Check if required representation is provided in other thread.
        for(size_t thread = 0; thread < config().size(); ++thread)
          if(thread != index)
            for(const auto& rp : config()[thread].representationProviders)
              if(rp.representation == requirement.representation)
              {
                if(provided)
                {
                  OUTPUT_ERROR(config()[index].name << ": Representation " << rp.representation << " is provided by multiple threads!");
                  return false;
                }
                provided = true;
                if(std::find(received[thread].begin(), received[thread].end(), std::string(requirement.representation)) == received[thread].end())
                  received[thread].emplace_back(requirement.representation);
              }
      }

      if(maybeAlias)
      {
        // Search representation in specific thread.
        for(const auto& rp : config()[aliasThreadIndex].representationProviders)
        {
          if(config()[aliasThreadIndex].name + rp.representation == requirement.representation)
          {
            if(provided)
            {
              OUTPUT_ERROR(config()[index].name << ": Representation " << requirement.representation << " is also provided as alias!");
              return false;
            }
            // Check if the representation and the alias are equal.
            if(!typeInfo.areTypesEqual(typeInfo, requirement.representation, rp.representation))
            {
              OUTPUT_ERROR(config()[index].name << ": The representations " << rp.representation << " and the alias " << requirement.representation << " are not equal!");
              return false;
            }
            provided = true;
            if(std::find(received[aliasThreadIndex].begin(), received[aliasThreadIndex].end(), std::string(requirement.representation)) == received[aliasThreadIndex].end())
              received[aliasThreadIndex].emplace_back(requirement.representation);
            break;
          }
        }
        if(!provided)
        {
          const std::string& rp(requirement.representation);
          OUTPUT_ERROR(config()[index].name << ": The representation " << rp.substr(config()[aliasThreadIndex].name.size(), rp.size())
                       << " that is expected from thread " << config()[aliasThreadIndex].name << " is not provided in this thread!");
          return false;
        }
      }

      if(!provided)
      {
        OUTPUT_ERROR(config()[index].name << ": No provider for required representation " << requirement.representation << " required by " << module->name);
        return false;
      }
    }
  }
  return true;
}

bool ModuleGraphCreator::update(In& stream)
{
  for(std::list<Provider>& providerList : providers)
    providerList.clear();

  for(auto& thread : sent)
    for(std::vector<const char*>& s : thread)
      s.clear();

  for(auto& thread : received)
    for(std::vector<const char*>& r : thread)
      r.clear();
  representationsToReset.clear();

  // Remove all markings
  for(std::vector<bool>& r : required)
    std::fill(r.begin(), r.end(), false);

  // Remember which representations were provided by which module before
  Configuration prevConfig(config);
  stream >> config;
  ASSERT(prevConfig().empty() || config().empty() || prevConfig().size() == config().size());

  // Checks whether a representation is provided more than once in a thread.
  for(Configuration::Thread& thread : config())
  {
    std::sort(thread.representationProviders.begin(), thread.representationProviders.end());
    auto duplicate = std::unique(thread.representationProviders.begin(), thread.representationProviders.end(),
                         [](Configuration::RepresentationProvider& thread, Configuration::RepresentationProvider& thread2) { return thread.representation == thread2.representation; });
    if(duplicate != thread.representationProviders.end())
    {
      OUTPUT_ERROR(thread.name << ": " << duplicate->representation << " is provided by more than one module!");
      return false;
    }
  }

  // Fill shared representations
  if(!calcShared(config))
    return false;

  // Fill the list of all providers
  for(std::size_t j = 0; j < config().size(); j++)
  {
    for(const auto& rp : config()[j].representationProviders)
    {
      auto module = modules.find(rp.provider);
      for(const ModuleBase::Info& i : module->second->getModuleInfo())
        if(i.update && rp.representation == i.representation)
        {
          providers[j].emplace_back(i.representation, module->second);
          break;
        }
      int index = static_cast<int>(std::distance(modules.begin(), module));
      required[j][index] = true;
    }
  }

  // Fill providedByDefault list
  std::vector<std::string> providedByDefault;
  for(const std::string& representation : config.defaultRepresentations)
  {
    providedByDefault.emplace_back(representation);
    // Alias of every thread is also "provided"
    for(const Configuration::Thread& thread : config())
      providedByDefault.emplace_back(thread.name + representation);
  }

  // Sort providers
  for(std::size_t i = 0; i < providers.size(); i++)
  {
    if(!sortProviders(providedByDefault, i))
      return false;
  }

  // Append all blackboard entries that are now provided by a different module or no module anymore.
  std::multimap<std::string, std::string> currentProviders;
  for(const Configuration::Thread& thread : config())
  {
    for(const auto& currentProvider : thread.representationProviders)
      currentProviders.emplace(currentProvider.representation, currentProvider.provider);
  }
  for(const Configuration::Thread& thread : prevConfig())
  {
    for(const auto& prevProvider : thread.representationProviders)
    {
      auto range = currentProviders.equal_range(prevProvider.representation);
      if(range.first != range.second)
      {
        for(auto i = range.first; i != range.second; ++i)
          if(i->second == prevProvider.provider)
            goto found;
        representationsToReset.emplace_back(prevProvider.representation);
      found:
        ;
      }
      else
      {
        // No longer provided -> "default" or "off"
        for(const std::string& representation : config.defaultRepresentations)
          if(representation == prevProvider.representation)
            representationsToReset.emplace_back(representation);
      }
    }
  }
  return true;
}

bool ModuleGraphCreator::sortProviders(const std::vector<std::string>& providedByDefault, std::size_t index)
{
  // The representations already provided. These are all that are received from the other thread
  // and the ones that are provided by default.
  std::vector<std::string> provided = providedByDefault;
  for(const auto& thread : received[index])
    for(const char* r : thread)
      provided.emplace_back(r);

  int remaining = static_cast<int>(providers[index].size()), // The number of entries not correct sequence so far.
      pushBackCount = remaining; // The number of push_backs still allowed. If zero, no valid sequence is possible.
  std::list<Provider>::iterator i = providers[index].begin();
  while(i != providers[index].end())
  {
    const std::vector<ModuleBase::Info>& info = i->moduleBase->getModuleInfo();
    auto j = info.cbegin();
    for(; j != info.cend(); ++j)
      if(!j->update && std::string(j->representation) != i->representation && std::find(provided.begin(), provided.end(), j->representation) == provided.end())
        break;
    if(j != info.cend()) // at least one requirement missing
    {
      if(pushBackCount) // still one left to try
      {
        providers[index].emplace_back(*i);
        i = providers[index].erase(i);
        --pushBackCount;
      }
      else // we checked all, none was satisfied
      {
        std::string text;
        for(const std::string& p : provided)
        {
          text += text == "" ? "" : ", ";
          text += p;
        }
        std::string text2 = "";
        while(i != providers[index].end())
        {
          text2 += text2.empty() ? "" : ", ";
          text2 += i->representation;
          ++i;
        }
        if(text.empty())
          OUTPUT_ERROR(config()[index].name << ": Requirements missing for providers for " << text2 << "!");
        else
          OUTPUT_ERROR(config()[index].name << ": Only found consistent providers for " << text <<
                       ".\nRequirements missing for providers for " << text2 << "!");
        return false;
      }
    }
    else // we found one with all requirements fulfilled
    {
      provided.emplace_back(i->representation); // add representation provided
      ++i; // continue with next provider
      --remaining; // we have one less to go,
      pushBackCount = remaining; // and the search starts again
    }
  }
  return true;
}

ModuleGraphCreator::ExecutionValues::ExecutionValues(std::vector<std::vector<const char*>>& received, std::vector<std::vector<const char*>>& sent,
                                                     std::vector<std::string>& representationsToReset, std::vector<ModuleRequired>& modules,
                                                     std::vector<Configuration::RepresentationProvider>& providers) :
  representationsToReset(representationsToReset), modules(modules), providers(providers)
{
  ASSERT(received.size() == sent.size());
  for(std::size_t i = 0; i < received.size(); i++)
  {
    this->received.emplace_back();
    for(const char*& r : received[i])
      this->received.back().vector.emplace_back(r);
    this->sent.emplace_back();
    for(const char*& s : sent[i])
      this->sent.back().vector.emplace_back(s);
  }
}

const ModuleGraphCreator::ExecutionValues ModuleGraphCreator::getExecutionValues(std::size_t index)
{
  std::vector<ExecutionValues::ModuleRequired> modulesRequired;
  auto it = modules.begin();
  for(size_t i = 0; i < modules.size(); i++)
    modulesRequired.emplace_back(it++->second->name, required[index][i]);
  std::vector<Configuration::RepresentationProvider> providerList;
  for(const Provider& provider : providers[index])
    providerList.emplace_back(provider.representation, provider.moduleBase->name);

  return ExecutionValues(received[index], sent[index], representationsToReset, modulesRequired, providerList);
}
