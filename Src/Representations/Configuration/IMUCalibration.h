#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(IMUCalibration,
{,
  (AngleAxisf)(AngleAxisf::Identity()) rotation,
  (Vector3f)(Vector3f::Zero()) gyroFactor,
  (Vector3f)(Vector3f::Zero()) accFactor,
  (Vector3f)(Vector3f::Zero()) accOffset,
  (bool)(false) isCalibrated,
  (unsigned)(0) serialNumberIMUCalibration,
});
