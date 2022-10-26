#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(IMUCalibration,
{,
  (AngleAxisf)(AngleAxisf::Identity()) rotation,
  (Vector3f)(Vector3f::Zero()) gyroFactor,
  (bool)(false) isCalibrated,
  (unsigned)(0) serialNumberIMUCalibration,
});
