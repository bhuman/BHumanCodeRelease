#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/EnumIndexedArray.h"

#include <array>

STREAMABLE(FsrSensorData,
{
  FsrSensorData();

  void draw(),

  (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(float, FsrSensors::FsrSensor), Legs::Leg)) pressures, /**< Values of the pressure sensors in each foot (in kg) */
  (ENUM_INDEXED_ARRAY(float, Legs::Leg)) totals, /**< Total mass pressing on the left foot (in kg) */
});

inline FsrSensorData::FsrSensorData()
{
  FOREACH_ENUM(Legs::Leg, leg)
  {
    pressures[leg].fill(SensorData::off);
    totals[leg] = 0.f;
  }
}
