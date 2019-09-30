/**
 * @file Tools/Module/ModuleGraphRunner.cpp
 *
 * Implementation of a class representing the module graph runner.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#include "ModuleGraphRunner.h"
#ifdef TARGET_ROBOT
#include "Platform/Time.h"
#endif

void ModuleGraphRunner::destroy()
{
  validConfiguration = false;
  for(Provider& m : providers)
    if(m.moduleState->instance)
    {
      delete m.moduleState->instance;
      m.moduleState->instance = 0;
    }
  providers.clear();
  sent.clear();
  received.clear();
}

void ModuleGraphRunner::update(In& stream)
{
  providers.clear();

  ModuleGraphCreator::ExecutionValues values;
  stream >> values;
  received = values.received;
  sent = values.sent;

  // Adds available modules and updates if they are needed
  for(const auto& module : values.modules)
  {
    auto i = modules.find(module.module);
    if(i != modules.end())
      i->second.required = module.required;
    else
    {
      ASSERT(allModules.find(module.module) != allModules.end());
      modules.emplace(module.module, allModules.find(module.module)->second).first->second.required = module.required;
    }
  }

  // Creating the provider list
  for(const auto& rp : values.providers)
  {
    const auto m = modules.find(rp.provider);
    ASSERT(m != modules.end());
    for(const ModuleBase::Info& i : m->second.module->getModuleInfo())
      if(i.update && rp.representation == i.representation)
      {
        providers.emplace_back(i.representation, &m->second, i.update);
        break;
      }
  }

  // Reset all blackboard entries that are now provided by a different module or no module anymore
  // Note: Needed to prevent function pointers from becoming invalid.
  for(const std::string& representation : values.representationsToReset)
    if(Blackboard::getInstance().exists(representation.c_str()))
      Blackboard::getInstance().reset(representation.c_str());

  // Delete all modules that are not required anymore
  for(auto& m : modules)
  {
    if(!m.second.required && m.second.instance)
    {
      delete m.second.instance;
      m.second.instance = 0;
    }
  }

  validConfiguration = true;
  stream >> nextTimestamp; // Use this timestamp after execute was called
  this->timestamp = 0; // Invalid until execute was called
}

void ModuleGraphRunner::execute()
{
  // Execute all providers in the given sequence
  for(Provider& p : providers)
  {
    ASSERT(p.moduleState->required);
    if(!p.moduleState->instance)
      p.moduleState->instance = p.moduleState->module->createNew();
#ifdef TARGET_ROBOT
    unsigned timestamp = Time::getCurrentSystemTime();
#endif
    if(p.moduleState->instance)
      p.update(*p.moduleState->instance);
#ifdef TARGET_ROBOT
    int duration = Time::getTimeSince(timestamp);
    if(timestamp > 110000 &&
       ((duration > 100 &&
         !Global::getDebugRequestTable().isActive("representation:JPEGImage") &&
         !Global::getDebugRequestTable().isActive("representation:CameraImage")) ||
        duration > 500))
      OUTPUT_ERROR("TIMING: providing " << p.representation << " took " << duration
                   << " ms at " << timestamp / 1000 - 100 << " s after start");
#endif
  }
  BH_TRACE;

  if(!timestamp) // Configuration changed recently?
  {
    // all representations must be constructed now, so we can receive data
    timestamp = nextTimestamp;
    for(auto& s : toSend)
      s.clear();
    for(std::size_t i = 0; i < sent.size(); i++)
      for(const std::string& s : sent[i].vector)
        toSend[i].emplace_back(&Blackboard::getInstance()[s.c_str()]);

    for(auto& r : toReceive)
      r.clear();
    for(std::size_t i = 0; i < received.size(); i++)
      for(const std::string& r : received[i].vector)
        toReceive[i].emplace_back(&Blackboard::getInstance()[r.c_str()]);
  }
}

void ModuleGraphRunner::readPacket(In& stream, const std::size_t index)
{
  unsigned timestamp;
  stream >> timestamp;
  // Communication is only possible if both sides are based on the same module request.
  if(timestamp == this->timestamp)
    for(Streamable* s : toReceive[index])
      stream >> *s;
  else
    stream.skip(10000000); // skip everything
}

void ModuleGraphRunner::writePacket(Out& stream, const std::size_t index) const
{
  stream << timestamp;
  for(const Streamable* s : toSend[index])
    stream << *s;
}
