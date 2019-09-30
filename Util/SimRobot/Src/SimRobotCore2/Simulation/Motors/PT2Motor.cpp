/**
* @file Simulation/Motors/ServoMotor.cpp
* Implementation of class PT2Motor
* @author Thomas Muender
*/

#include <cmath>

#include "PT2Motor.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/Actuators/Joint.h"
#include "Simulation/Axis.h"
#include "CoreModule.h"
#include "Platform/Assert.h"

PT2Motor::PT2Motor()
{
  Simulation::simulation->scene->actuators.push_back(this);

  positionSensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  positionSensor.dimensions.push_back(1);
}

void PT2Motor::create(Joint* joint)
{
  ASSERT(dJointGetType(joint->joint) == dJointTypeHinge);
  this->joint = positionSensor.joint = joint;
  dJointSetHingeParam(joint->joint, dParamFMax, F);
}

void PT2Motor::act()
{
  lastSetpoints.push_back(this->setpoint);

  if(lastSetpoints.size() < 3) return;

  const float dt = Simulation::simulation->scene->stepLength;
  float y = static_cast<float>(dJointGetHingeAngle(joint->joint));

  ASSERT(T != 0.0);
  const float dx = dt / T * (K * lastSetpoints[0] - y - 2 * D * x);
  x += dx;
  float yd = std::fmin(x / T, V);//convert x to velocity and limit it
  x = yd * T;//convert limited velocity back to x to limit x as well

  lastSetpoints.pop_front();

  dJointSetHingeParam(joint->joint, dParamVel, yd);
}

void PT2Motor::setValue(float value)
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

bool PT2Motor::getMinAndMax(float& min, float& max) const
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

void PT2Motor::PositionSensor::updateValue()
{
  data.floatValue = static_cast<float>(dJointGetHingeAngle(joint->joint));
}

bool PT2Motor::PositionSensor::getMinAndMax(float& min, float& max) const
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

void PT2Motor::registerObjects()
{
  positionSensor.unit = unit = QString::fromUtf8("°");
  positionSensor.fullName = joint->fullName + ".position";
  fullName = joint->fullName + ".position";

  CoreModule::application->registerObject(*CoreModule::module, positionSensor, joint);
  CoreModule::application->registerObject(*CoreModule::module, *this, joint);
}
