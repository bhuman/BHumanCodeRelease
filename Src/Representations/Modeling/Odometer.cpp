/*
 * @file Odometer.cpp
 *
 * Some additional odometry information
 *
 * @author Tim Laue
 */

#include "Odometer.h"
#include "Platform/BHAssert.h"
#include <cmath>

void Odometer::verify() const
{
  ASSERT(std::isfinite(distanceWalked));
  ASSERT(odometryOffset.isFinite());
}
