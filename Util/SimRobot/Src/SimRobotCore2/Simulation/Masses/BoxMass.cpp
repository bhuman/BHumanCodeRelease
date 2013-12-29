/**
* @file Simulation/Masses/BoxMass.cpp
* Implementation of class BoxMass
* @author Colin Graf
*/

#include "BoxMass.h"

void BoxMass::assembleMass()
{
  dMassSetBoxTotal(&mass, value, depth, width, height);
}
