/**
* @file Simulation/Masses/InertiaMatrixMass.cpp
* Implementation of class InertiaMatrixMass
* @author Colin Graf
*/

#include "InertiaMatrixMass.h"

void InertiaMatrixMass::assembleMass()
{
  dMassSetParameters(&mass, value, x, y, z, ixx, iyy, izz, ixy, ixz, iyz);
}
