#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Motion/SensorData.h"

/**
 * Encapsulates the joint sensor data as it is provided by NAOqi.
 * Without variance.
 */
STREAMABLE_WITH_BASE(JointSensorData, JointAngles,
{
  ENUM(TemperatureStatus,
  {,
    regular,
    hot,
    veryHot,
    criticallyHot,
  });

  JointSensorData(),

  (ENUM_INDEXED_ARRAY(short, Joints::Joint)) currents, /**< The currents of all motors. */
  (ENUM_INDEXED_ARRAY(unsigned char, Joints::Joint)) temperatures, /**< The currents of all motors. */
  (ENUM_INDEXED_ARRAY(JointSensorData::TemperatureStatus, Joints::Joint)) status, /**< The status of all motors. */
});

inline JointSensorData::JointSensorData() :
  JointAngles()
{
  currents.fill(static_cast<short>(SensorData::off));
  temperatures.fill(0);
  status.fill(TemperatureStatus::regular);
}
