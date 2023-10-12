/**
 * @file MotionProvider.cpp
 *
 * This file implements a module that provides representations from the motion
 * thread for the current frame.
 *
 * @author Arne Hasselbring
 */

#include "MotionProvider.h"

MAKE_MODULE(MotionProvider);

void MotionProvider::update(OdometryData& odometryData)
{
  odometryData = static_cast<const OdometryData&>(theMotionOdometryData);
}
