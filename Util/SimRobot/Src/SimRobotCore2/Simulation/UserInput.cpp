/**
 * @file Simulation/UserInput.h
 * Implemention of class to forward user input. An actuator port can be set
 * by the user and a controller can read that value through a sensor port.
 * @author Thomas Röfer
 */

#include "UserInput.h"
#include "CoreModule.h"
#include "SensorWidget.h"

const QIcon* UserInput::InputPort::getIcon() const
{
  return &CoreModule::module->actuatorIcon;
}

SimRobot::Widget* UserInput::InputPort::createWidget()
{
  CoreModule::module->application->openObject(CoreModule::module->actuatorsObject);
  if(ActuatorsWidget::actuatorsWidget)
    ActuatorsWidget::actuatorsWidget->openActuator(fullName);
  return nullptr;
}

void UserInput::InputPort::setValue(float value)
{
  if(value < min)
    data.floatValue = min;
  else if(value > max)
    data.floatValue = max;
  else
    data.floatValue = value;
}

const QIcon* UserInput::OutputPort::getIcon() const
{
  return &CoreModule::module->sensorIcon;
}

SimRobot::Widget* UserInput::OutputPort::createWidget()
{
  return new SensorWidget(this);
}

void UserInput::registerObjects()
{
  inputPort.data.floatValue = inputPort.defaultValue;
  inputPort.fullName = fullName + ".value";
  outputPort.input = &inputPort;
  CoreModule::application->registerObject(*CoreModule::module, inputPort, this);
  CoreModule::application->registerObject(*CoreModule::module, outputPort, this);
}

const QIcon* UserInput::getIcon() const
{
  return &CoreModule::module->sliderIcon;
}
