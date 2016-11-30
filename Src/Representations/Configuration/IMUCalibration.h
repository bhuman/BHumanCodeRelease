#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Streams/AutoStreamable.h"

struct IMUCalibration : public Streamable
{
  Quaternionf rotation;

private:
  virtual void serialize(In* in, Out* out);
};


inline void IMUCalibration::serialize(In* in, Out* out)
{
  Vector3a rotation = Rotation::AngleAxis::pack(AngleAxisf(this->rotation)).cast<Angle>();

  STREAM_REGISTER_BEGIN
  STREAM(rotation)
  STREAM_REGISTER_FINISH

  if(in)
    this->rotation = Quaternionf(Rotation::AngleAxis::unpack(rotation.cast<float>()));
}
