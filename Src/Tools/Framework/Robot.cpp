/**
 * @file Tools/Framework/Robot.cpp
 *
 * This file implements the class Robot as a list of threads.
 *
 * @author Jan Fiedler
 */

#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/LocalRobot.h"

#include <QString>
#include <QFileInfo>
#endif

#include "Robot.h"
#include "Threads/Debug.h"
#include "Tools/Framework/ModuleContainer.h"
#include "Tools/FunctionList.h"

Robot::Robot(const std::string& name) : name(name)
{
  Global::theSettings = new Settings();
  InMapFile stream("threads.cfg");
  delete Global::theSettings;

  Configuration config;
  stream >> config;
  if(!stream.exists() || config().empty())
    FAIL("Cannot open the file threads.cfg or the file is empty.");

  // Here also the settings are initialized for the file access.
  push_back(new Debug(config));

#ifdef TARGET_SIM
  // add and connect Simulator
  push_back(new LocalRobot(static_cast<Debug*>(front())));
  robotThread = static_cast<LocalRobot*>(back());
  ASSERT(robotThread);
  front()->setGlobals();
#endif

  // Logger uses Global of Debug here
  logger = new Logger(config);

  // start threads
  for(size_t i = 0; i < config().size(); i++)
    push_back(new ModuleContainer(config, i, logger));

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

#ifdef TARGET_SIM
extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  FunctionList::execute();
  QFileInfo info(simRobot.getFilePath());
  QString baseName = info.baseName();
  return new ConsoleRoboCupCtrl(simRobot);
}

void Robot::update()
{
  robotThread->update();
}
#endif
