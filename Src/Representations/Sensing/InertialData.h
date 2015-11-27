#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * The inertialData contains filtered data from the IMU.
 */
STREAMABLE_WITH_BASE(InertialData, InertialSensorData,
{,
  (Quaternionf)(Quaternionf::Identity()) orientation, /** The orientation of the torso represented as a quaternion. */
});
