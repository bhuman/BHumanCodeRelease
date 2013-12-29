/**
* @file SimRobotHelp/HelpModule.cpp
* @author Colin Graf
*/

#include <QString>

#include "HelpModule.h"
#include "HelpWidget.h"

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  return new HelpModule(simRobot);
}

SimRobot::Application* HelpModule::application;
HelpModule* HelpModule::module;

HelpModule::HelpModule(SimRobot::Application& application)
{
  this->module = this;
  this->application = &application;
}

bool HelpModule::compile()
{
  application->registerObject(*this, helpObject, 0);
  return true;
}
