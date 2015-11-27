/**
 * @file CameraInfo.cpp
 * Implementation of struct CameraInfo
 */

#include "CameraInfo.h"
#include "Tools/Math/BHMath.h"

void CameraInfo::updateFocalLength()
{
  focalLength = width / (2.f * std::tan(openingAngleWidth / 2.f));
  focalLengthInv = 1.f / focalLength;
  focalLenPow2 = focalLength * focalLength;
}
