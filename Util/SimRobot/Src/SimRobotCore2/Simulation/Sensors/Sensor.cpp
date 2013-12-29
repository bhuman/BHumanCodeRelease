/**
* @file Simulation/Sensor.cpp
* Implementation of class Sensor
* @author Colin Graf
*/

#include "Sensor.h"
#include "CoreModule.h"
#include "SensorWidget.h"

const QIcon* Sensor::Port::getIcon() const
{
  return &CoreModule::module->sensorIcon;
}

SimRobot::Widget* Sensor::Port::createWidget()
{
  return new SensorWidget(this);
}

SimRobotCore2::SensorPort::Data Sensor::Port::getValue()
{
  if(lastSimulationStep != Simulation::simulation->simulationStep)
  {
    updateValue();
    lastSimulationStep = Simulation::simulation->simulationStep;
  }
  return data;
}
