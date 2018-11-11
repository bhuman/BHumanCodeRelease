/**
* @file Simulation/Geometries/BoxGeometry.h
* Declaration of class BoxGeometry
* @author Colin Graf
*/

#pragma once

#include "Simulation/Geometries/Geometry.h"

/**
* @class BoxGeometry
* A box shaped geometry
*/
class BoxGeometry : public Geometry
{
public:
  float width; /**< The width of the box (cy) */
  float height; /**< The height of the box (cz) */
  float depth; /**< The depth of the box (cx) */

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
