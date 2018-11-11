/**
* @file Simulation/PhysicalObject.h
* Declaration of class PhysicalObject
* @author Colin Graf
*/

#pragma once

#include "Simulation/SimObject.h"
#include "Tools/Math/Pose3f.h"

class Body;

/**
* @class PhysicalObject
* Abstract class for scene graph objects with physical representation
*/
class PhysicalObject : public SimObject
{
public:
  PhysicalObject* parent; /**< The only parent of the primary object (or \c 0 in case that this is the root object) */
  Body* parentBody; /**< The superior body object (might be 0) */

  Pose3f pose; /**< The absolute pose of the object */
  std::list<PhysicalObject*> physicalChildren; /**< List of subordinate physical scene graph objects */
  std::list<PhysicalObject*> physicalDrawings; /**< List of subordinate physical objects that will be drawn relative to this one */

  /** Default constructor */
  PhysicalObject() : parent(0), parentBody(0) {}

  /**
  * Creates the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

protected:
  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

private:
  std::list<SimRobotCore2::Controller3DDrawing*> controllerDrawings; /**< Drawings registered by another SimRobot module */

protected:
  // API
  virtual bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing);
  virtual bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing);
  virtual SimRobotCore2::Body* getParentBody();
};
