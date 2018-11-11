/**
* @file Simulation/Appearances/SphereAppearance.h
* Declaration of class SphereAppearance
* @author Colin Graf
*/

#pragma once

#include "Simulation/Appearances/Appearance.h"

/**
* @class SphereAppearance
* The graphical representation of a sphere
*/
class SphereAppearance : public Appearance
{
public:
  float radius; /**< The radius of the sphere */

private:
  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;
};
