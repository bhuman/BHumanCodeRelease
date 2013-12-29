/**
* @file Simulation/PhysicalObject.h
* Implementation of class PhysicalObject
* @author Colin Graf
*/

#include "Simulation/PhysicalObject.h"
#include "Platform/Assert.h"
#include "Simulation/Body.h"

void PhysicalObject::addParent(Element& element)
{
  ASSERT(!parent);
  parent = dynamic_cast<PhysicalObject*>(&element);
  parent->physicalChildren.push_back(this);
  parent->physicalDrawings.push_back(this);
  SimObject::addParent(element);
}

void PhysicalObject::createPhysics()
{
  // find parent body for child objects
  Body* body = dynamic_cast<Body*>(this);
  if(!body)
    body = parentBody;

  // initialize and call createPhysics() for each child object
  for(std::list<PhysicalObject*>::const_iterator iter = physicalChildren.begin(), end = physicalChildren.end(); iter != end; ++iter)
  {
    // compute pose of child object
    PhysicalObject* object = *iter;
    object->pose = pose;
    if(object->translation)
      object->pose.translate(*object->translation);
    if(object->rotation)
      object->pose.rotate(*object->rotation);

    //
    object->parentBody = body;
    object->createPhysics();
  }
}

void PhysicalObject::drawPhysics(unsigned int flags) const
{
  if(flags & SimRobotCore2::Renderer::showControllerDrawings)
    for(std::list<SimRobotCore2::Controller3DDrawing*>::const_iterator iter = controllerDrawings.begin(), end = controllerDrawings.end(); iter != end; ++iter)
      (*iter)->draw();
  for(std::list<PhysicalObject*>::const_iterator iter = physicalDrawings.begin(), end = physicalDrawings.end(); iter != end; ++iter)
    (*iter)->drawPhysics(flags);
}

bool PhysicalObject::registerDrawing(SimRobotCore2::Controller3DDrawing& drawing)
{
  controllerDrawings.push_back(&drawing);
  return true;
}

bool PhysicalObject::unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing)
{
  for(std::list<SimRobotCore2::Controller3DDrawing*>::iterator iter = controllerDrawings.begin(), end = controllerDrawings.end(); iter != end; ++iter)
    if(*iter == &drawing)
    {
      controllerDrawings.erase(iter);
      return true;
    }
  return false;
}

SimRobotCore2::Body* PhysicalObject::getParentBody()
{
  return parentBody;
}
