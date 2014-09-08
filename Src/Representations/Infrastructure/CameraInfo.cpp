/**
* @file CameraInfo.cpp
* Implementation of class CameraInfo
*/

#include "CameraInfo.h"
#include "Tools/Settings.h"
#include "Tools/Global.h"
#include "Tools/Math/BHMath.h"

void CameraInfo::updateFocalLength()
{
  focalLength = width / (2.f * std::tan(this->openingAngleWidth / 2.f));
  focalLengthInv = 1.0f / focalLength;
  focalLenPow2 = focalLength * focalLength;
}

void CameraInfo::serialize(In* in, Out* out)
{
  float openingAngleWidth = toDegrees(this->openingAngleWidth);
  float openingAngleHeight = toDegrees(this->openingAngleHeight);

  STREAM_REGISTER_BEGIN;
  STREAM(camera);
  STREAM(width);
  STREAM(height);
  STREAM(openingAngleWidth);
  STREAM(openingAngleHeight);
  STREAM(opticalCenter);
  STREAM_REGISTER_FINISH;

  if(in)
  {
    this->openingAngleWidth = fromDegrees(openingAngleWidth);
    this->openingAngleHeight = fromDegrees(openingAngleHeight);
    updateFocalLength();
  }
}
