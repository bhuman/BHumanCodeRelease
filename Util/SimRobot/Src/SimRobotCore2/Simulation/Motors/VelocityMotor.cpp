/**
* @file Simulation/Motors/VelocityMotor.cpp
* Implementation of class VelocityMotor
* @author Colin Graf
* @author Thomas Röfer
*/

#include <cmath>

#include "Simulation/Motors/VelocityMotor.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Actuators/Joint.h"
#include "Simulation/Axis.h"
#include "CoreModule.h"
#include "Platform/Assert.h"

VelocityMotor::VelocityMotor() : maxVelocity(0), maxForce(0)
{
  Simulation::simulation->scene->actuators.push_back(this);

  positionSensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  positionSensor.dimensions.push_back(1);

  velocitySensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  velocitySensor.dimensions.push_back(1);
}

void VelocityMotor::create(Joint* joint)
{
  ASSERT(dJointGetType(joint->joint) == dJointTypeHinge || dJointGetType(joint->joint) == dJointTypeSlider);
  this->joint = positionSensor.joint = velocitySensor.joint = joint;
  velocitySensor.maxVelocity = maxVelocity;
  if(dJointGetType(joint->joint) == dJointTypeHinge)
    dJointSetHingeParam(joint->joint, dParamFMax, maxForce);
  else
    dJointSetSliderParam(joint->joint, dParamFMax, maxForce);
}

void VelocityMotor::act()
{
  dJointSetHingeParam(joint->joint, dParamVel, setpoint);
}

void VelocityMotor::setValue(float value)
{
  if(value > maxVelocity)
    setpoint = maxVelocity;
  else if(value < -maxVelocity)
    setpoint = -maxVelocity;
  else
    setpoint = value;
}

bool VelocityMotor::getMinAndMax(float& min, float& max) const
{
  min = -maxVelocity;
  max = maxVelocity;
  return true;
}

void VelocityMotor::PositionSensor::updateValue()
{
  data.floatValue = (dJointGetType(joint->joint) == dJointTypeHinge
                     ? static_cast<float>(dJointGetHingeAngle(joint->joint))
                     : static_cast<float>(dJointGetSliderPosition(joint->joint))) + (joint->axis->deflection ? joint->axis->deflection->offset : 0.f);
}

bool VelocityMotor::PositionSensor::getMinAndMax(float& min, float& max) const
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

void VelocityMotor::VelocitySensor::updateValue()
{
  data.floatValue = dJointGetType(joint->joint) == dJointTypeHinge
                    ? static_cast<float>(dJointGetHingeParam(joint->joint, dParamVel))
                    : static_cast<float>(dJointGetSliderParam(joint->joint, dParamVel));
}

bool VelocityMotor::VelocitySensor::getMinAndMax(float& min, float& max) const
{
  min = -maxVelocity;
  max = maxVelocity;
  return true;
}

void VelocityMotor::registerObjects()
{
  if(dJointGetType(joint->joint) == dJointTypeHinge)
  {
    positionSensor.unit = QString::fromUtf8("°");
    velocitySensor.unit = unit = QString::fromUtf8("°/s");
  }
  else
  {
    positionSensor.unit = "m";
    velocitySensor.unit = unit = "m/s";
  }

  positionSensor.fullName = joint->fullName + ".position";
  CoreModule::application->registerObject(*CoreModule::module, positionSensor, joint);

  velocitySensor.fullName = joint->fullName + ".velocity";
  CoreModule::application->registerObject(*CoreModule::module, velocitySensor, joint);

  fullName = joint->fullName + ".velocity";
  CoreModule::application->registerObject(*CoreModule::module, *this, joint);
}
