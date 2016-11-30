#pragma once

#include "Tools/Motion/SensorData.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SystemSensorData,
{
  void draw(),

  (float)(SensorData::off) cpuTemperature, /** The temperature of the cpu (in °C). */
  (float)(SensorData::off) batteryCurrent, /** The current of the battery (in A). */
  (float)(SensorData::off) batteryLevel, /** The current of the battery (in %). Range: [0.0, 1.0] */
  (float)(SensorData::off) batteryTemperature, /** The temperature of the battery (in %, whatever that means...). Range: [0.0, 1.0] */
  (bool)(false) batteryCharging, /** The battery is charging */
});
