/**
* @file Simulation/Appearances/CylinderAppearance.h
* Declaration of class CylinderAppearance
* @author Colin Graf
*/

#pragma once

#include "Simulation/Appearances/Appearance.h"

/**
* @class CylinderAppearance
* The graphical representation of a cylinder
*/
class CylinderAppearance : public Appearance
{
public:
  float height; /**< The height of the cylinder */
  float radius; /**< The radius */

private:
  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;
};
