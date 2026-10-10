#pragma once

#include "Tools/Motion/SensorData.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(SystemSensorData,
{
  void draw(),

  (float)(SensorData::off) cpuTemperature, /**< The temperature of the CPU (in °C). */
  (float)(SensorData::off) gpuTemperature, /**< The temperature of the GPU (in °C). */
  (float)(SensorData::off) batteryLevel, /**< The current of the battery (in %). Range: [0.0, 1.0] */
  (bool)(false) emergencyMode, /**< The robot is in emergency mode */
});
