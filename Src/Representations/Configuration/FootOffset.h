/**
 * @file FootOffset.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"

STREAMABLE(FootValues,
{,
  (float) left,
  (float) right,
});

STREAMABLE(FootOffset,
{,
  (float) forward,
  (float) backward,
  (FootValues) leftFoot,
  (FootValues) rightFoot,
  (bool) isCalibrated,
});
