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
  void createPhysics() override;

  /**
  * Creates a stationary ODE geometry
  * @param parentPose The pose of the group or geometry
  * @param geometry A geometry description
  * @param callback A collision callback function attached to the geometry
  */
  void addGeometry(const Pose3f& parentPose, Geometry& geometry, SimRobotCore2::CollisionCallback* callback);

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  void drawPhysics(unsigned int flags) const override;

  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

private:
  // API
  const QString& getFullName() const override {return SimObject::getFullName();}
  SimRobot::Widget* createWidget() override {return SimObject::createWidget();}
  const QIcon* getIcon() const override {return SimObject::getIcon();}
  SimRobotCore2::Renderer* createRenderer() override {return SimObject::createRenderer();}
  bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing) override {return ::PhysicalObject::registerDrawing(drawing);}
  bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing) override {return ::PhysicalObject::unregisterDrawing(drawing);}
  SimRobotCore2::Body* getParentBody() override {return ::PhysicalObject::getParentBody();}
};
