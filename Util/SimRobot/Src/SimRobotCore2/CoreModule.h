/**
* @file CoreModule.h
* Declaration of class CoreModule
* @author Colin Graf
*/

#pragma once

#include <QIcon>

#include "../SimRobot/SimRobot.h"
#include "Simulation/Simulation.h"
#include "ActuatorsWidget.h"

class SimObject;

/**
* @class CoreModule
* The SimRobotCore Module for simulating robots and their environment
*/
class CoreModule : public SimRobot::Module, public Simulation
{
public:
  static SimRobot::Application* application;
  static CoreModule* module;

  QIcon sceneIcon;
  QIcon objectIcon;
  QIcon sensorIcon;
  QIcon actuatorIcon;
  QIcon hingeIcon;
  QIcon sliderIcon;
  QIcon appearanceIcon;
  ActuatorsObject actuatorsObject;

  /**
  * Constructor
  * @param application The interface to the SimRobot application that loaded the module
  */
  CoreModule(SimRobot::Application& application);

private:
  /**
  * Called to initialize the module. In this phase the module can do the following tasks
  *   - registering its own objects to the scene graph (using \c Application::registerObject)
  *   - adding status labels to the GUI (using \c Application::addStatusLabel)
  *   - suggest or load further modules (using \c Application::registerModule, \c Application::loadModule)
  * @return Whether an error occurred while initializing the module or not
  */
  bool compile() override;

  /** Called to perform another simulation step */
  void update() override;
};
