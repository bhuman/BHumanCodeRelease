/**
* @file Simulation/GraphicalObject.h
* Declaration of class GraphicalObject
* @author Colin Graf
*/

#pragma once

#include "Simulation/SimObject.h"

/**
* @class GraphicalObject
* Abstract class for scene graph objects with graphical representation or subordinate graphical representation
*/
enum SurfaceColor : unsigned char;
class GraphicalObject
{
public:
  std::list<GraphicalObject*> graphicalDrawings; /**< List of subordinate graphical scene graph objects */

  /** Default constructor */
  GraphicalObject() : initializedContexts(0), listId(0) {}

  /**
  * Prepares the object and the currently selected OpenGL context for drawing the object.
  * Loads textures and creates display lists. Hence, this function is called for each OpenGL
  * context the object should be drawn in.
  */
  virtual void createGraphics();

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  virtual void assembleAppearances(SurfaceColor color) const;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (as fast as possible) */
  virtual void drawAppearances(SurfaceColor color, bool drawControllerDrawings) const;

protected:
  unsigned int initializedContexts;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  // API
  virtual bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing);
  virtual bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing);

private:
  unsigned int listId; /**< The display list created for this object */
  std::list<SimRobotCore2::Controller3DDrawing*> controllerDrawings; /**< Drawings registered by another SimRobot module */
};
