/**
* @file CameraInfo.cpp
* Implementation of class CameraInfo
*/

#include "CameraInfo.h"

void CameraInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(camera);
  STREAM(width);
  STREAM(height);
  STREAM(openingAngleWidth);
  STREAM(openingAngleHeight);
  STREAM(opticalCenter);
  if(in)
  {
    focalLength = width / (2.f * std::tan(openingAngleWidth / 2.f));
    focalLengthInv = 1.0f / focalLength;
    focalLenPow2 = focalLength * focalLength;
    focalLenPow4 = focalLenPow2 * focalLenPow2;
  }
  STREAM_REGISTER_FINISH;
}
