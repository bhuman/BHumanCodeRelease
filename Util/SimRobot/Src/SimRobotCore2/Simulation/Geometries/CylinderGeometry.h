/**
* @file Simulation/Geometries/CylinderGeometry.h
* Declaration of class CylinderGeometry
* @author Colin Graf
*/

#pragma once

#include "Simulation/Geometries/Geometry.h"

/**
* @class CylinderGeometry
* A cylinder shaped geometry
*/
class CylinderGeometry : public Geometry
{
public:
  float height; /**< The height of the cylinder */
  float radius; /**< The radius of the sphere */

private:
  /**
  * Creates the geometry (not including \c translation and \c rotation)
  * @param space A space to create the geometry in
  * @param The created geometry
  */
  dGeomID createGeometry(dSpaceID space) override;

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  void drawPhysics(unsigned int flags) const override;
};
