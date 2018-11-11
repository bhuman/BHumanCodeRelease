/**
* @file Simulation/Actuator.cpp
* Implementation of class Actuator
* @author Colin Graf
*/

#include "Actuator.h"
#include "CoreModule.h"

void Actuator::addParent(Element& element)
{
  ::PhysicalObject::addParent(element);
}

const QIcon* Actuator::Port::getIcon() const
{
  return &CoreModule::module->actuatorIcon;
}

SimRobot::Widget* Actuator::Port::createWidget()
{
  CoreModule::module->application->openObject(CoreModule::module->actuatorsObject);
  if(ActuatorsWidget::actuatorsWidget)
    ActuatorsWidget::actuatorsWidget->openActuator(fullName);
  return nullptr;
}
