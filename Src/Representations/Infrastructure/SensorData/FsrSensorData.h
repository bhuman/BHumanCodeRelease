#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/Streams/EnumIndexedArray.h"

#include <array>

STREAMABLE(FsrSensorData,
{
  FsrSensorData();

  void draw(),

  (ENUM_INDEXED_ARRAY(float, (FsrSensors) FsrSensor)) left, /**< Values of the four pressure sensors in the left foot (in kg) */
  (ENUM_INDEXED_ARRAY(float, (FsrSensors) FsrSensor)) right, /**< Values of the four pressure sensors in the right foot (in kg) */
  (float)(0.f) leftTotal, /**< Total mass pressing on the left foot (in kg) */
  (float)(0.f) rightTotal, /**< Total mass pressing on the right foot (in kg) */
});

inline FsrSensorData::FsrSensorData()
{
  left.fill(SensorData::off);
  right.fill(SensorData::off);
}
