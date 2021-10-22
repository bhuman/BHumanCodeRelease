/**
 * @file Tools/Framework/Robot.cpp
 *
 * This file implements the class Robot as a list of threads.
 *
 * @author Jan Fiedler
 */

#include "Robot.h"
#include "Platform/BHAssert.h"
#include "Threads/Debug.h"
#include "Tools/Framework/Configuration.h"
#include "Tools/Framework/ModuleContainer.h"
#include "Tools/Global.h"
#include "Tools/Streams/InStreams.h"

Robot::Robot(const Settings& settings, const std::string& name) : name(name)
{
  Global::theSettings = const_cast<Settings*>(&settings);
  InMapFile stream("threads.cfg");
  Global::theSettings = nullptr;

  Configuration config;
  stream >> config;
  if(!stream.exists() || config().empty())
    FAIL("Cannot open the file threads.cfg or the file is empty.");

  push_back(new Debug(settings, name, config));

  // Logger uses Global of Debug here
  logger = new Logger(config);

  // start threads
  for(std::size_t i = 0; i < config().size(); i++)
    push_back(new ModuleContainer(settings, name, config, i, logger));

  // connect sender and receiver
  for(const Configuration::Thread& con : config())
  {
    ModuleContainer* thread = static_cast<ModuleContainer*>(lookupThread(con.name));
    for(const Configuration::Thread& other : config())
    {
      ModuleContainer* otherThread = static_cast<ModuleContainer*>(lookupThread(other.name));
      if(thread != otherThread)
        thread->connectWithSender(otherThread, config);
    }

    // connect every Thread with Debug
    thread->connectWithDebug(static_cast<Debug*>(front()), con);
  }
}
