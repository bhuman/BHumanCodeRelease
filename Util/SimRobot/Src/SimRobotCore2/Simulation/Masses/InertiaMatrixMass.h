/**
* @file Simulation/Masses/InertiaMatrixMass.h
* Declaration of class InertiaMatrixMass
* @author Colin Graf
*/

#pragma once

#include "Mass.h"

/**
* @class Mass
* A mass defined by its center and the inertia matrix relative to the origin
*/
class InertiaMatrixMass : public Mass
{
public:
  float value; /**< The total mass */
  float x; /**< The center of mass x */
  float y; /**< The center of mass y */
  float z; /**< The center of mass z */
  float ixx;
  float ixy;
  float ixz;
  float iyy;
  float iyz;
  float izz;

private:
  /** Creates the mass (not including children, \c translation or \c rotation) */
  void assembleMass() override;
};
