/**
* @file Simulation/Appearances/BoxAppearance.h
* Declaration of class BoxAppearance
* @author Colin Graf
*/

#pragma once

#include "Simulation/Appearances/Appearance.h"

/**
* @class BoxAppearance
* The graphical representation of a simple box
*/
class BoxAppearance : public Appearance
{
public:
  float width; /**< The width of the box (cy) */
  float height; /**< The height of the box (cz) */
  float depth; /**< The depth of the box (cx) */

private:
  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;
};
