/**
* @file Simulation/Geometries/SphereGeometry.h
* Declaration of class SphereGeometry
* @author Colin Graf
*/

#pragma once

#include "Simulation/Geometries/Geometry.h"

/**
* @class SphereGeometry
* A sphere shaped geometry
*/
class SphereGeometry : public Geometry
{
public:
  float radius; /**< The radius of the sphere */

private:
  /**
  * Creates the geometry (not including \c translation and \c rotation)
  * @param space A space to create the geometry in
  * @param The created geometry
  */
  virtual dGeomID createGeometry(dSpaceID space);

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;
};
