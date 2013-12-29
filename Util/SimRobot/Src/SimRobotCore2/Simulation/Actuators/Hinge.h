/**
* @file Simulation/Joints/Hinge.h
* Declaration of class Hinge
* @author Colin Graf
*/

#pragma once

#include "Joint.h"

/**
* @class Hinge
* A hinge joint that connects two bodies
*/
class Hinge : public Joint
{
private:
  /**
  * Creates the physical objects used by the OpenDynamicsEngine (ODE).
  * These are a geometry object for collision detection and/or a body,
  * if the simulation object is movable.
  */
  virtual void createPhysics();

  //API
  virtual const QIcon* getIcon() const;
};
