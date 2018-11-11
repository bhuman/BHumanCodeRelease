/**
* @file Simulation/Masses/BoxMass.h
* Declaration of class BoxMass
* @author Colin Graf
*/

#pragma once

#include "Mass.h"

/**
* @class Mass
* The mass of a simple box
*/
class BoxMass : public Mass
{
public:
  float value; /**< The total mass of the box */
  float width; /**< The width of the box (cy) */
  float height; /**< The height of the box (cz) */
  float depth; /**< The depth of the box (cx) */

private:
  /** Creates the mass (not including children, \c translation or \c rotation) */
  void assembleMass() override;
};
