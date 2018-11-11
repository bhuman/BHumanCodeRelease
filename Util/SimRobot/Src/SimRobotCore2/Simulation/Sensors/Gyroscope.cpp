/**
* @file Simulation/Sensors/Gyroscope.cpp
* Implementation of class Gyroscope
* @author Colin Graf
*/

#include "Simulation/Sensors/Gyroscope.h"
#include "Simulation/Body.h"
#include "Platform/Assert.h"
#include "Tools/ODETools.h"
#include "CoreModule.h"

Gyroscope::Gyroscope()
{
  sensor.sensorType = SimRobotCore2::SensorPort::floatArraySensor;
  sensor.unit = QString::fromUtf8("°/s");
  sensor.descriptions.append("x");
  sensor.descriptions.append("y");
  sensor.descriptions.append("z");
  sensor.dimensions.append(3);
  sensor.data.floatArray = sensor.angularVel;
}

void Gyroscope::createPhysics()
{
  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;
}

void Gyroscope::addParent(Element& element)
{
  sensor.body = dynamic_cast<Body*>(&element);
  ASSERT(sensor.body);
  Sensor::addParent(element);
}

void Gyroscope::registerObjects()
{
  sensor.fullName = fullName + ".angularVelocities";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void Gyroscope::GyroscopeSensor::updateValue()
{
  const dReal* angularVelInWorld = dBodyGetAngularVel(body->body);
  dVector3 result;
  dBodyVectorFromWorld(body->body, angularVelInWorld[0], angularVelInWorld[1], angularVelInWorld[2], result);
  Vector3f angularVelInBody;
  ODETools::convertVector(result, angularVelInBody);
  const Vector3f angularVelInSensor = offset.rotation.inverse() * angularVelInBody;
  angularVel[0] = angularVelInSensor.x();
  angularVel[1] = angularVelInSensor.y();
  angularVel[2] = angularVelInSensor.z();
}
