/**
 * @file FootSoleRotationCalibration.h
 * This representation provides the calibration values for the foot soles
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Angle.h"
#include "Streaming/AutoStreamable.h"
#include "RobotParts/Legs.h"
#include "Streaming/EnumIndexedArray.h"
#include "Streaming/Function.h"

STREAMABLE(FeetPair,
{,
  (Vector2a)(Vector2a(0, 0)) rotationOffset,
  (bool)(false) isCalibrated,
});

STREAMABLE(FootSoleRotationCalibration,
{
  FUNCTION(void()) calibrateSoleRotation,

  (int)(0) id, /**< Id of the calibration. This helps so the InertialDataProvider only needs to convert the values once, in case they change. */
  (ENUM_INDEXED_ARRAY(FeetPair, Legs::Leg)) footCalibration, /**< The rotation calibration values for the feet. */
  (bool)(false) notCalibratable, /**< Something is not working, which prevents a calibration. */
});
