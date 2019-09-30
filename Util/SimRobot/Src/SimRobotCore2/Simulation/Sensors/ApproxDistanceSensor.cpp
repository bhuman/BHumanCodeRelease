/**
* @file Simulation/Sensors/ApproxDistanceSensor.cpp
* Implementation of class ApproxDistanceSensor
* @author Colin Graf
*/

#include "Platform/OpenGL.h"
#include <cmath>

#include "Simulation/Sensors/ApproxDistanceSensor.h"
#include "Simulation/Geometries/Geometry.h"
#include "Platform/Assert.h"
#include "Tools/OpenGLTools.h"
#include "Tools/ODETools.h"
#include "CoreModule.h"
#include <algorithm>

ApproxDistanceSensor::ApproxDistanceSensor()
{
  sensor.sensorType = SimRobotCore2::SensorPort::floatSensor;
  sensor.unit = "m";
}

void ApproxDistanceSensor::createPhysics()
{
  OpenGLTools::convertTransformation(rotation, translation, transformation);

  sensor.tanHalfAngleX = std::tan(angleX * 0.5f);
  sensor.tanHalfAngleY = std::tan(angleY * 0.5f);
  float width = sensor.tanHalfAngleX * max * 2.f;
  float height = sensor.tanHalfAngleY * max * 2.f;
  float depth = max;
  sensor.geom = dCreateBox(Simulation::simulation->rootSpace, depth, width, height);
  sensor.scanRayGeom = dCreateRay(Simulation::simulation->rootSpace, max);
  sensor.min = min;
  sensor.max = max;
  sensor.maxSqrDist = max * max;
  if(translation)
    sensor.offset.translation = *translation;
  if(rotation)
    sensor.offset.rotation = *rotation;
}

void ApproxDistanceSensor::registerObjects()
{
  sensor.fullName = fullName + ".distance";
  CoreModule::application->registerObject(*CoreModule::module, sensor, this);

  Sensor::registerObjects();
}

void ApproxDistanceSensor::addParent(Element& element)
{
  sensor.physicalObject = dynamic_cast<::PhysicalObject*>(&element);
  ASSERT(sensor.physicalObject);
  Sensor::addParent(element);
}

void ApproxDistanceSensor::DistanceSensor::staticCollisionCallback(ApproxDistanceSensor::DistanceSensor* sensor, dGeomID geom1, dGeomID geom2)
{
  ASSERT(geom1 == sensor->geom);
  ASSERT(!dGeomIsSpace(geom2));

  Geometry* geometry = static_cast<Geometry*>(dGeomGetData(geom2));
  if(reinterpret_cast<::PhysicalObject*>(geometry->parentBody) == sensor->physicalObject)
    return; // avoid detecting the body on which the sensor is mounted

  const dReal* pos = dGeomGetPosition(geom2);
  Vector3f geomPos;
  ODETools::convertVector(pos, geomPos);
  const float approxSqrDist = (geomPos - sensor->pose.translation).squaredNorm() - geometry->innerRadiusSqr;
  if(approxSqrDist >= sensor->closestSqrDistance)
    return; // we already found another geometrie that was closer

  const Vector3f relPos = sensor->invertedPose * geomPos;
  if(relPos.x() <= 0.f)
    return; // center of the geometry should be in front of the distance sensor

  const float halfMaxY = sensor->tanHalfAngleX * relPos.x();
  const float halfMaxZ = sensor->tanHalfAngleY * relPos.x();
  if(std::max(std::abs(relPos.y()) - geometry->outerRadius, 0.f) >= halfMaxY || std::max(std::abs(relPos.z()) - geometry->outerRadius, 0.f) >= halfMaxZ)
    return; // the sphere that covers the geometry does not collide with the pyramid of the distance sensor

  if(std::max(std::abs(relPos.y()) - geometry->innerRadius, 0.f) < halfMaxY && std::max(std::abs(relPos.z()) - geometry->innerRadius, 0.f) < halfMaxZ)
    goto hit; // the sphere enclosed by the geometry collides with the pyramid of the distance sensor

  // geom2 might collide with the pyramid of the distance sensor. let us perform a hit scan along one of the pyramid's sides to find out..
  {
    const Vector3f scanDir = sensor->pose.rotation * Vector3f(relPos.x(), std::max(std::min(relPos.y(), halfMaxY), -halfMaxY), std::max(std::min(relPos.z(), halfMaxZ), -halfMaxZ));
    const Vector3f& sensorPos = sensor->pose.translation;
    dGeomRaySet(sensor->scanRayGeom, sensorPos.x(), sensorPos.y(), sensorPos.z(), scanDir.x(), scanDir.y(), scanDir.z());
    dContactGeom contactGeom;
    if(dCollide(sensor->scanRayGeom, geom2, CONTACTS_UNIMPORTANT | 1, &contactGeom, sizeof(dContactGeom)) <= 0)
      return;
  }

hit:
  sensor->closestSqrDistance = approxSqrDist;
  sensor->closestGeom = geom2;
}

void ApproxDistanceSensor::DistanceSensor::staticCollisionWithSpaceCallback(ApproxDistanceSensor::DistanceSensor* sensor, dGeomID geom1, dGeomID geom2)
{
  ASSERT(geom1 == sensor->geom);
  ASSERT(dGeomIsSpace(geom2));
  dSpaceCollide2(geom1, geom2, sensor, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
}

void ApproxDistanceSensor::DistanceSensor::updateValue()
{
  pose = physicalObject->pose;
  pose.conc(offset);
  invertedPose = pose.inverse();
  Vector3f boxPos = pose * Vector3f(max * 0.5f, 0.f, 0.f);
  dGeomSetPosition(geom, boxPos.x(), boxPos.y(), boxPos.z());
  dMatrix3 matrix3;
  ODETools::convertMatrix(pose.rotation, matrix3);
  dGeomSetRotation(geom, matrix3);
  closestGeom = 0;
  closestSqrDistance = maxSqrDist;
  dSpaceCollide2(geom, reinterpret_cast<dGeomID>(Simulation::simulation->movableSpace), this, reinterpret_cast<dNearCallback*>(&staticCollisionWithSpaceCallback));
  dSpaceCollide2(geom, reinterpret_cast<dGeomID>(Simulation::simulation->staticSpace), this, reinterpret_cast<dNearCallback*>(&staticCollisionCallback));
  if(closestGeom)
  {
    const dReal* pos = dGeomGetPosition(closestGeom);
    Geometry* geometry = static_cast<Geometry*>(dGeomGetData(closestGeom));
    data.floatValue = (Vector3f(static_cast<float>(pos[0]), static_cast<float>(pos[1]), static_cast<float>(pos[2])) - pose.translation).norm() - geometry->innerRadius;
    if(data.floatValue < min)
      data.floatValue = min;
  }
  else
    data.floatValue = max;
}

bool ApproxDistanceSensor::DistanceSensor::getMinAndMax(float& min, float& max) const
{
  min = this->min;
  max = this->max;
  return true;
}

void ApproxDistanceSensor::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showSensors)
  {
    const Vector3f ml(max, -std::tan(angleX * 0.5f) * max, 0);
    const Vector3f mt(max, 0, std::tan(angleY * 0.5f) * max);
    const Vector3f tl(max, ml.y(), mt.z());
    const Vector3f tr(max, -ml.y(), mt.z());
    const Vector3f bl(max, ml.y(), -mt.z());
    const Vector3f br(max, -ml.y(), -mt.z());

    glBegin(GL_LINE_LOOP);
      glColor3f(0.5f, 0, 0);
      glNormal3f (0, 0, 1.f);
      glVertex3f(tl.x(), tl.y(), tl.z());
      glVertex3f(tr.x(), tr.y(), tr.z());
      glVertex3f(br.x(), br.y(), br.z());
      glVertex3f(bl.x(), bl.y(), bl.z());
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(tl.x(), tl.y(), tl.z());
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(tr.x(), tr.y(), tr.z());
    glEnd();
    glBegin(GL_LINE_STRIP);
      glVertex3f(bl.x(), bl.y(), bl.z());
      glVertex3f(0.f, 0.f, 0.f);
      glVertex3f(br.x(), br.y(), br.z());
    glEnd();
  }

  Sensor::drawPhysics(flags);
  glPopMatrix();
}
