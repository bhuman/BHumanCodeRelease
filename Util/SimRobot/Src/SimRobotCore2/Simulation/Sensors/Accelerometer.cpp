/**
* @file Simulation/Sensors/Accelerometer.cpp
* Implementation of class Accelerometer
* @author Colin Graf
*/

#include "Simulation/Sensors/Accelerometer.h"
#include "Simulation/Scene.h"
#include "Simulation/Body.h"
#include "Platform/Assert.h"
#include "Tools/ODETools.h"
#include "CoreModule.h"

Accelerometer::Accelerometer()
{
  sensor.sensorType = SimRobotCore2::SensorPort::floatArraySensor;
  sensor.unit = "m/s\xb2";
  sensor.descriptions.append("x");
  sensor.descriptions.append("y");
  sensor.descriptions.append("z");
  sensor.dimensions.append(3);
  sensor.data.floatArray = sensor.linearAcc;
  sensor.linearAcc[0] = sensor.linearAcc[1] = sensor.linearAcc[2] = 0.f;
  sensor.linearVelInWorld[0] = sensor.linearVelInWorld[1] = sensor.linearVelInWorld[2] = 0.f;
}

void Accelerometer::createPhysics()
{
  sensor.offset.translation = -sensor.body->centerOfMass;
  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;
}

void Accelerometer::addParent(Element& element)
{
  sensor.body = dynamic_cast<Body*>(&element);
  ASSERT(sensor.body);
  Sensor::addParent(element);
}

void Accelerometer::registerObjects()
{
  sensor.fullName = fullName + ".acceleration";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void Accelerometer::AccelerometerSensor::updateValue()
{
  dVector3 result;
  dBodyGetRelPointVel(body->body, offset.translation.x(), offset.translation.y(), offset.translation.z(), result);
  Vector3f linearVelInWorld;
  ODETools::convertVector(result, linearVelInWorld);

  Scene* scene = Simulation::simulation->scene;
  const float timeScale = 1.f / (scene->stepLength * static_cast<float>(Simulation::simulation->simulationStep - lastSimulationStep));

  float linearAccInWorld[3];
  linearAccInWorld[0] = static_cast<float>((linearVelInWorld[0] - this->linearVelInWorld[0]) * timeScale);
  linearAccInWorld[1] = static_cast<float>((linearVelInWorld[1] - this->linearVelInWorld[1]) * timeScale);
  linearAccInWorld[2] = static_cast<float>((linearVelInWorld[2] - this->linearVelInWorld[2]) * timeScale - scene->gravity);

  dBodyVectorFromWorld(body->body, linearAccInWorld[0], linearAccInWorld[1], linearAccInWorld[2], result);
  Vector3f linearAccInBody;
  ODETools::convertVector(result, linearAccInBody);
  const Vector3f linearAccInSensor = offset.rotation.inverse() * linearAccInBody;
  linearAcc[0] = linearAccInSensor.x();
  linearAcc[1] = linearAccInSensor.y();
  linearAcc[2] = linearAccInSensor.z();

  this->linearVelInWorld[0] = static_cast<float>(linearVelInWorld[0]);
  this->linearVelInWorld[1] = static_cast<float>(linearVelInWorld[1]);
  this->linearVelInWorld[2] = static_cast<float>(linearVelInWorld[2]);
  lastSimulationStep = Simulation::simulation->simulationStep;
}
