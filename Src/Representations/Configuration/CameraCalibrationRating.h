/**
 * Defines a representation that contains a rating for the camera calibration.
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Function.h"

STREAMABLE(CameraCalibrationRating,
{
  FUNCTION(void(const bool& curCamIsUpper)) setResolution;
  FUNCTION(void(const bool& curCamIsUpper)) check;
  FUNCTION(void()) evaluate;
  FUNCTION(void()) evalOutput,

  (bool)(false) isGood,
  (bool)(false) wasEvaluated,
  (bool)(false) wasAborted,
});
