/**
* @file Simulation/Appearances/CapsuleAppearance.h
* Declaration of class CapsuleAppearance
* @author Colin Graf
*/

#pragma once

#include "Simulation/Appearances/Appearance.h"

/**
* @class CapsuleAppearance
* The graphical representation of a capsule
*/
class CapsuleAppearance : public Appearance
{
public:
  float height; /**< The height of the capsule */
  float radius; /**< The radius */

private:
  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  void assembleAppearances(SurfaceColor color) const override;
};
