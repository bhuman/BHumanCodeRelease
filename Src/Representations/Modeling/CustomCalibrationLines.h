/**
 * @file CustomCalibrationLines.h
 *
 * Contains information about custom lines used for manual camera calibration.
 *
 * @author Adam Cihasev
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"

STREAMABLE(CustomCalibrationLines,
{
  ENUM(Type,
  {,
    undefined,
    closeGoalAreaConnectingLine,
    innerGoalAreaParallelLine,
    outerGoalAreaParallelLine,
    farGoalAreaConnectingLine,
  });
  ,
  (ENUM_INDEXED_ARRAY(LinesPercept::Line, Type)) lines, /**< The custom calibration lines. */
});
