/**
 * @file FsrData.h
 *  This representation provides the calibrated FsrData, represented as percentual values
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Motion/SensorData.h"
#include "RobotParts/FsrSensors.h"
#include "RobotParts/Legs.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(SolePressureInfo,
{,
  (unsigned int)(0) forwardPressure, // Last time stamp the toe had pressure
  (unsigned int)(0) backwardPressure, // Last time stamp the heel had pressure
  (unsigned int)(0) leftPressure, // Last time stamp the left side had pressure
  (unsigned int)(0) rightPressure, // Last time stamp the right side had pressure
  (unsigned int)(0) hasPressure, // Last time stamp when foot had pressure
  (unsigned int)(0) hasPressureSince, // Time stamp the last time the foot regained pressure
  (float)(0.f) sagittalRatio, // Ratio between forward and backward pressure
  (float)(0.f) lateralRatio, // Ratio between left and right pressure
  (float)(0.f) totals, /**< % mass pressing on the feet, based on the max measured pressure */
});

STREAMABLE(FsrData,
{
  FsrData(),

  (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(float, FsrSensors::FsrSensor), Legs::Leg)) pressures, /**< Values of the pressure sensors in each foot (in % based on the max measured pressure) */
  (ENUM_INDEXED_ARRAY(SolePressureInfo, Legs::Leg)) legInfo,
  (float)(0.f) minPressure, /**< % min pressure, to assume ground contact. */
  (bool)(false) isCalibrated,
  (unsigned int)(0) lastUpdateTimestamp, /**< Time stamp when FsrData was last updated. */
});

inline FsrData::FsrData()
{
  FOREACH_ENUM(Legs::Leg, leg)
  {
    pressures[leg].fill(SensorData::off);
  }
}
