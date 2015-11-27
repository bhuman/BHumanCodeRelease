/**
* @file Simulation/Motors/ServoMotor.cpp
* Implementation of class ServoMotor
* @author Colin Graf
*/

#include <cmath>

#include "Simulation/Motors/ServoMotor.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Actuators/Joint.h"
#include "Simulation/Axis.h"
#include "CoreModule.h"
#include "Platform/Assert.h"

ServoMotor::ServoMotor() : maxVelocity(0), maxForce(0)
{
  Simulation::simulation->scene->actuators.push_back(this);

  positionSensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  positionSensor.dimensions.push_back(1);
}

void ServoMotor::create(Joint* joint)
{
  ASSERT(dJointGetType(joint->joint) == dJointTypeHinge || dJointGetType(joint->joint) == dJointTypeSlider);
  this->joint = positionSensor.joint = joint;
  if(dJointGetType(joint->joint) == dJointTypeHinge)
    dJointSetHingeParam(joint->joint, dParamFMax, maxForce);
  else
    dJointSetSliderParam(joint->joint, dParamFMax, maxForce);
}

void ServoMotor::act()
{
  const float currentPos = (dJointGetType(joint->joint) == dJointTypeHinge
                            ? (float) dJointGetHingeAngle(joint->joint)
                            : (float) dJointGetSliderPosition(joint->joint)) + (joint->axis->deflection ? joint->axis->deflection->offset : 0.f);

  float setpoint = this->setpoint;
  const float maxValueChange = maxVelocity * Simulation::simulation->scene->stepLength;
  if(std::abs(setpoint - currentPos) > maxValueChange)
  {
    if(setpoint < currentPos)
      setpoint = currentPos - maxValueChange;
    else
      setpoint = currentPos + maxValueChange;
  }

  const float newVel = controller.getOutput(currentPos, setpoint);
  if(dJointGetType(joint->joint) == dJointTypeHinge)
    dJointSetHingeParam(joint->joint, dParamVel, dReal(newVel));
  else
    dJointSetSliderParam(joint->joint, dParamVel, dReal(newVel));
}

float ServoMotor::Controller::getOutput(float currentPos, float setpoint)
{
  const float deltaTime = Simulation::simulation->scene->stepLength;
  const float error = setpoint - currentPos;
  errorSum += i * error * deltaTime;
  float result = p * error + errorSum + (d * (error - lastError)) / deltaTime;
  lastError = error;
  return result;
}

void ServoMotor::setValue(float value)
{
  setpoint = value;
  Axis::Deflection* deflection = joint->axis->deflection;
  if(deflection)
  {
    if(setpoint > deflection->max)
      setpoint = deflection->max;
    else if(setpoint < deflection->min)
      setpoint = deflection->min;
  }
}

bool ServoMotor::getMinAndMax(float& min, float& max) const
{
  Axis::Deflection* deflection = joint->axis->deflection;
  if(deflection)
  {
    min = deflection->min;
    max = deflection->max;
    return true;
  }
  return false;
}

void ServoMotor::PositionSensor::updateValue()
{
  data.floatValue = (dJointGetType(joint->joint) == dJointTypeHinge
                     ? (float) dJointGetHingeAngle(joint->joint)
                     : (float) dJointGetSliderPosition(joint->joint)) + (joint->axis->deflection ? joint->axis->deflection->offset : 0.f);
}

bool ServoMotor::PositionSensor::getMinAndMax(float& min, float& max) const
{
  Axis::Deflection* deflection = joint->axis->deflection;
  if(deflection)
  {
    min = deflection->min;
    max = deflection->max;
    return true;
  }
  return false;
}

void ServoMotor::registerObjects()
{
  if(dJointGetType(joint->joint) == dJointTypeHinge)
    positionSensor.unit = unit = QString::fromUtf8("°");
  else
    positionSensor.unit = unit = "m";
  positionSensor.fullName = joint->fullName + ".position";
  fullName = joint->fullName + ".position";

  CoreModule::application->registerObject(*CoreModule::module, positionSensor, joint);
  CoreModule::application->registerObject(*CoreModule::module, *this, joint);
}
