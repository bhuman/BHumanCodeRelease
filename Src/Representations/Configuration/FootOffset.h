/**
 * @file FootOffset.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"
#include "Math/Angle.h"

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
