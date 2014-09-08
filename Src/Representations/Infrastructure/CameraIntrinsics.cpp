#include "CameraIntrinsics.h"

void CameraIntrinsics::serialize(In* in, Out* out)
{
  float upperOpeningAngleWidth = toDegrees(this->upperOpeningAngleWidth);
  float upperOpeningAngleHeight = toDegrees(this->upperOpeningAngleHeight);
  float lowerOpeningAngleWidth = toDegrees(this->lowerOpeningAngleWidth);
  float lowerOpeningAngleHeight = toDegrees(this->lowerOpeningAngleHeight);

  STREAM_REGISTER_BEGIN;
  STREAM(upperOpeningAngleWidth);
  STREAM(upperOpeningAngleHeight);
  STREAM(upperOpticalCenter);
  STREAM(lowerOpeningAngleWidth);
  STREAM(lowerOpeningAngleHeight);
  STREAM(lowerOpticalCenter);
  STREAM_REGISTER_FINISH;

  if(in)
  {
    this->upperOpeningAngleWidth = fromDegrees(upperOpeningAngleWidth);
    this->upperOpeningAngleHeight = fromDegrees(upperOpeningAngleHeight);
    this->lowerOpeningAngleWidth = fromDegrees(lowerOpeningAngleWidth);
    this->lowerOpeningAngleHeight = fromDegrees(lowerOpeningAngleHeight);
  }
}