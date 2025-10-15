/**
 * @file SolePressureState.h
 * This representation provides information about the pressure of the soles.
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Motion/SensorData.h"
#include "RobotParts/FsrSensors.h"
#include "RobotParts/Legs.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(SolePressureInfo,
{,
  (bool)(false) hasPressure, // Last time stamp when foot had pressure
  (unsigned int)(0) hasPressureSince, // Time stamp the last time the foot regained pressure
  (float)(0.f) totals, /**< % mass pressing on the feet, based on the max measured pressure */
});

STREAMABLE(SolePressureState,
{
  SolePressureState(),

  (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(float, FsrSensors::FsrSensor), Legs::Leg)) pressures, /**< Values of the pressure sensors in each foot (in % based on the max measured pressure) */
  (ENUM_INDEXED_ARRAY(SolePressureInfo, Legs::Leg)) legInfo,
  (float)(0.f) minPressure, /**< % min pressure, to assume ground contact. */
  (bool)(false) isCalibrated,
});

inline SolePressureState::SolePressureState()
{
  FOREACH_ENUM(Legs::Leg, leg)
  {
    pressures[leg].fill(SensorData::off);
  }
}
