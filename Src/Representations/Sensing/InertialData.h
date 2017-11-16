#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * The inertialData contains filtered data from the IMU.
 */
STREAMABLE_WITH_BASE(InertialData, InertialSensorData,
{
  void draw(),

  (Quaternionf)(Quaternionf::Identity()) orientation2D, /** The orientation of the torso represented as a quaternion without the z-Rotation. */
  (Quaternionf)(Quaternionf::Identity()) orientation3D, /** The orientation of the torso represented as a quaternion including the z-Rotation. */
  (Vector3f)(Vector3f::Identity()) filteredAcc,
});
