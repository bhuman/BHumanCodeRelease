/**
* @file Simulation/Geometries/Geometry.cpp
* Implementation of class Geometry
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Geometries/Geometry.h"
#include "Tools/OpenGLTools.h"
#include "Platform/Assert.h"

Geometry::Geometry() : immaterial(false), material(0), collisionCallbacks(0), created(false)
{
  color[0] = color[1] = color[2] = 0.8f;
  color[3] = 1.0f;
}

Geometry::~Geometry()
{
  if(collisionCallbacks)
    delete collisionCallbacks;
}

void Geometry::addParent(Element& element)
{
  ::PhysicalObject::addParent(element);
}

dGeomID Geometry::createGeometry(dSpaceID space)
{
  if(!created)
  {
    OpenGLTools::convertTransformation(rotation, translation, transformation);
    created = true;
  }
  return 0;
}

void Geometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);
  ::PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}

bool Geometry::registerCollisionCallback(SimRobotCore2::CollisionCallback& collisionCallback)
{
  if(!collisionCallbacks)
    collisionCallbacks = new std::list<SimRobotCore2::CollisionCallback*>();
  collisionCallbacks->push_back(&collisionCallback);
  return false;
}

bool Geometry::unregisterCollisionCallback(SimRobotCore2::CollisionCallback& collisionCallback)
{
  for(std::list<SimRobotCore2::CollisionCallback*>::iterator iter = collisionCallbacks->begin(), end = collisionCallbacks->end(); iter != end; ++iter)
    if(*iter == &collisionCallback)
    {
      collisionCallbacks->erase(iter);
      if(collisionCallbacks->empty())
      {
        delete collisionCallbacks;
        collisionCallbacks = 0;
      }
      return true;
    }
  return false;
}

void Geometry::Material::addParent(Element& element)
{
  Geometry* geometry = dynamic_cast<Geometry*>(&element);
  ASSERT(!geometry->material);
  geometry->material = this;
}

bool Geometry::Material::getFriction(const Material& other, float& friction) const
{
  {
    std::unordered_map<const Material*, float>::const_iterator iter = materialToFriction.find(&other);
    if(iter != materialToFriction.end())
    {
      friction = iter->second;
      return friction >= 0.f;
    }
  }

  friction = 0.f;
  int frictionValues = 0;

  {
    std::unordered_map<std::string, float>::const_iterator iter = frictions.find(other.name);
    if(iter != frictions.end())
    {
      friction += iter->second;
      ++frictionValues;
    }
  }

  {
    std::unordered_map<std::string, float>::const_iterator iter = other.frictions.find(name);
    if(iter != other.frictions.end())
    {
      friction += iter->second;
      ++frictionValues;
    }
  }

  bool frictionDefined = frictionValues > 0;
  if(frictionDefined)
    friction /= float(frictionValues);
  else
    friction = -1.f;

  materialToFriction[&other] = friction;
  return frictionDefined;
}

bool Geometry::Material::getRollingFriction(const Material& other, float& rollingFriction) const
{
  {
    std::unordered_map<const Material*, float>::const_iterator iter = materialToRollingFriction.find(&other);
    if(iter != materialToRollingFriction.end())
    {
      rollingFriction = iter->second;
      return rollingFriction >= 0.f;
    }
  }

  {
    std::unordered_map<std::string, float>::const_iterator iter = rollingFrictions.find(other.name);
    if(iter != rollingFrictions.end())
    {
      rollingFriction = iter->second;
      materialToRollingFriction[&other] = rollingFriction;
      return true;
    }
  }

  rollingFriction = -1.f;
  materialToRollingFriction[&other] = rollingFriction;
  return false;
}
