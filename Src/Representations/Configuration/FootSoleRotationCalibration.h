#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FootSoleRotationCalibration,
{,
  (Angle) yRotationOffset,
  (bool) isCalibrated,
});
