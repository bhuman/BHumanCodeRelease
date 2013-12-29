/**
* @file Simulation/Compound.h
* Declaration of class Compound
* @author Colin Graf
*/

#pragma once

#include "Simulation/PhysicalObject.h"
#include "Simulation/GraphicalObject.h"
#include "Simulation/Geometries/Geometry.h"

/**
* @class Compound
* A non-movable physical object
*/
class Compound : public PhysicalObject, public GraphicalObject, public SimRobotCore2::Compound
{
private:
  /**
  * Creates the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  /**
  * Creates a stationary ODE geometry
  * @param parentPose The pose of the group or geometry
  * @param geometry A geometry description
  * @param callback A collision callback function attached to the geometry
  */
  void addGeometry(const Pose3<>& parentPose, Geometry& geometry, SimRobotCore2::CollisionCallback* callback);

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  virtual void assembleAppearances() const;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

private:
  // API
  virtual const QString& getFullName() const {return SimObject::getFullName();}
  virtual SimRobot::Widget* createWidget() {return SimObject::createWidget();}
  virtual const QIcon* getIcon() const {return SimObject::getIcon();}
  virtual SimRobotCore2::Renderer* createRenderer() {return SimObject::createRenderer();}
  virtual bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing) {return ::PhysicalObject::registerDrawing(drawing);}
  virtual bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing) {return ::PhysicalObject::unregisterDrawing(drawing);}
  virtual SimRobotCore2::Body* getParentBody() {return ::PhysicalObject::getParentBody();}
};
