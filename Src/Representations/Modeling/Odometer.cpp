/*
 * @file Odometer.cpp
 *
 * Some additional odometry information
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "Odometer.h"
#include "Platform/BHAssert.h"
#include <cmath>

void Odometer::verify() const
{
  ASSERT(!std::isnan(distanceWalked));
  ASSERT(!std::isnan(static_cast<float>(odometryOffset.rotation)));
  ASSERT(!std::isnan(odometryOffset.translation.x()));
  ASSERT(!std::isnan(odometryOffset.translation.y()));
}
