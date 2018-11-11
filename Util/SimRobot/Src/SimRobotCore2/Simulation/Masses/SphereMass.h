/**
* @file Simulation/Masses/SphereMass.h
* Declaration of class SphereMass
* @author Colin Graf
*/

#pragma once

#include "Mass.h"

/**
* @class Mass
* The mass of a sphere
*/
class SphereMass : public Mass
{
public:
  float value; /**< The total mass of the box */
  float radius; /**< The radius of the sphere */

private:
  /** Creates the mass (not including children, \c translation or \c rotation) */
  void assembleMass() override;
};
