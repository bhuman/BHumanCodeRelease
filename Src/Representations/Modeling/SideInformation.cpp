/**
 * @file Representations/Modeling/SideInformation.cpp
 *
 * Implementation a struct that contains information about the field half the robot seems to be currently in.
 *
 * @author Tim Laue
 */

#include "SideInformation.h"
#include "Platform/BHAssert.h"
#include <cmath>

void SideInformation::verify() const
{
  ASSERT(!(robotMustBeInOpponentHalf && robotMustBeInOwnHalf));
  ASSERT(std::isfinite(largestXCoordinatePossible));
}
