/**
* @file Simulation/GraphicalObject.cpp
* Implementation of class GraphicalObject
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "Simulation/GraphicalObject.h"
#include "Platform/Assert.h"

void GraphicalObject::createGraphics()
{
  ++initializedContexts;
  for(std::list<GraphicalObject*>::const_iterator iter = graphicalDrawings.begin(), end = graphicalDrawings.end(); iter != end; ++iter)
  {
    GraphicalObject* graphicalObject = *iter;
    if(graphicalObject->initializedContexts != initializedContexts)
    {
      graphicalObject->createGraphics();
      graphicalObject->initializedContexts = initializedContexts;
    }
  }

  // create display list
  unsigned int listId = glGenLists(1);
  ASSERT(listId > 0);
  ASSERT(this->listId == 0 || this->listId == listId);
  this->listId = listId;
  glNewList(listId, GL_COMPILE);
    assembleAppearances(SurfaceColor::ownColor);
  glEndList();
}

void GraphicalObject::drawAppearances(SurfaceColor color, bool drawControllerDrawings) const
{
  if(drawControllerDrawings)
    for(std::list<SimRobotCore2::Controller3DDrawing*>::const_iterator iter = controllerDrawings.begin(), end = controllerDrawings.end(); iter != end; ++iter)
      (*iter)->draw();
  else if(color == ownColor)
  {
    ASSERT(listId);
    glCallList(listId);
  }
  else
    assembleAppearances(color);
}

void GraphicalObject::assembleAppearances(SurfaceColor color) const
{
  for(std::list<GraphicalObject*>::const_iterator iter = graphicalDrawings.begin(), end = graphicalDrawings.end(); iter != end; ++iter)
    (*iter)->drawAppearances(color, false);
}

void GraphicalObject::addParent(Element& element)
{
  dynamic_cast<GraphicalObject*>(&element)->graphicalDrawings.push_back(this);
}

bool GraphicalObject::registerDrawing(SimRobotCore2::Controller3DDrawing& drawing)
{
  controllerDrawings.push_back(&drawing);
  return true;
}

bool GraphicalObject::unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing)
{
  for(std::list<SimRobotCore2::Controller3DDrawing*>::iterator iter = controllerDrawings.begin(), end = controllerDrawings.end(); iter != end; ++iter)
    if(*iter == &drawing)
    {
      controllerDrawings.erase(iter);
      return true;
    }
  return false;
}
