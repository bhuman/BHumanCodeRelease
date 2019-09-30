/**
* @file Simulation/Sensors/SingleDistanceSensor.cpp
* Implementation of class SingleDistanceSensor
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Sensors/SingleDistanceSensor.h"
#include "Simulation/Body.h"
#include "Platform/Assert.h"
#include "Tools/OpenGLTools.h"
#include "CoreModule.h"

SingleDistanceSensor::SingleDistanceSensor()
{
  sensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  sensor.unit = "m";
}

void SingleDistanceSensor::createPhysics()
{
  OpenGLTools::convertTransformation(rotation, translation, transformation);

  sensor.geom = dCreateRay(Simulation::simulation->rootSpace, max);
  sensor.min = min;
  sensor.max = max;
  sensor.maxSqrDist = max * max;
  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;
}

void SingleDistanceSensor::registerObjects()
{
  sensor.fullName = fullName + ".distance";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void SingleDistanceSensor::addParent(Element& element)
{
  sensor.physicalObject = dynamic_cast< ::PhysicalObject*>(&element);
  ASSERT(sensor.physicalObject);
  Sensor::addParent(element);
}

void SingleDistanceSensor::DistanceSensor::staticCollisionCallback(SingleDistanceSensor::DistanceSensor* sensor, dGeomID geom1, dGeomID geom2)
{
  ASSERT(geom1 == sensor->geom);
  ASSERT(!dGeomIsSpace(geom2));

  dContactGeom contactGeoms[4];
  int contacts = dCollide(geom1, geom2, 4, contactGeoms, sizeof(dContactGeom));

  for(int i = 0; i < contacts; ++i)
  {
    const dContactGeom& contactGeom = contactGeoms[i];
    const float sqrDistance = (Vector3f(static_cast<float>(contactGeom.pos[0]), static_cast<float>(contactGeom.pos[1]), static_cast<float>(contactGeom.pos[2])) - sensor->pose.translation).squaredNorm();
    if(sqrDistance < sensor->closestSqrDistance)
    {
      sensor->closestSqrDistance = sqrDistance;
      sensor->closestGeom = geom2;
    }
  }
}

void SingleDistanceSensor::DistanceSensor::staticCollisionWithSpaceCallback(SingleDistanceSensor::DistanceSensor* sensor, dGeomID geom1, dGeomID geom2)
{
  ASSERT(geom1 == sensor->geom);
  ASSERT(dGeomIsSpace(geom2));
  dSpaceCollide2(geom1, geom2, sensor, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
}

void SingleDistanceSensor::DistanceSensor::updateValue()
{
  pose = physicalObject->pose;
  pose.conc(offset);
  const Vector3f& pos = pose.translation;
  const Vector3f dir = pose.rotation.col(0);
  dGeomRaySet(geom, pos.x(), pos.y(), pos.z(), dir.x(), dir.y(), dir.z());
  closestGeom = 0;
  closestSqrDistance = maxSqrDist;
  dSpaceCollide2(geom, reinterpret_cast<dGeomID>(Simulation::simulation->movableSpace), this, reinterpret_cast<dNearCallback*>(&staticCollisionWithSpaceCallback));
  dSpaceCollide2(geom, reinterpret_cast<dGeomID>(Simulation::simulation->staticSpace), this, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
  if(closestGeom)
  {
    data.floatValue = std::sqrt(closestSqrDistance);
    if(data.floatValue < min)
      data.floatValue = min;
  }
  else
    data.floatValue = max;
}

bool SingleDistanceSensor::DistanceSensor::getMinAndMax(float& min, float& max) const
{
  min = this->min;
  max = this->max;
  return true;
}

void SingleDistanceSensor::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showSensors)
  {
    glBegin(GL_LINES);
      glColor3f(0.5f, 0, 0);
      glNormal3f (0, 0, 1.f);
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(max, 0.f, 0.f);
    glEnd();
  }

  Sensor::drawPhysics(flags);
  glPopMatrix();
}
