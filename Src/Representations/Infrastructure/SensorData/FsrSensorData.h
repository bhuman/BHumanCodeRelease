#pragma once

#include "Tools/SensorData.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

#include <array>

STREAMABLE(FsrSensorData,
{
  ENUM(FsrSensorPosition,
  {,
    fl,
    fr,
    bl,
    br,
  });

  FsrSensorData();
  ,
  (std::array<float, FsrSensorData::numOfFsrSensorPositions>) left, /**< Values of the four pressure sensors in the left foot (in kg) */
  (std::array<float, FsrSensorData::numOfFsrSensorPositions>) right, /**< Values of the four pressure sensors in the right foot (in kg) */
  (float)(0.f) leftTotal, /**< Total mass pressing on the left foot (in kg) */
  (float)(0.f) rightTotal, /**< Total mass pressing on the right foot (in kg) */
});

inline FsrSensorData::FsrSensorData()
{
  left.fill(SensorData::off);
  right.fill(SensorData::off);
}