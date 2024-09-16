#pragma once

#include "Representations/Sensing/InertialSensorData.h"
#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * The inertialData contains filtered data from the IMU.
 */
STREAMABLE_WITH_BASE(InertialData, InertialSensorData,
{
  void draw(),

  (Quaternionf)(Quaternionf::Identity()) orientation2D, /**< The orientation of the torso represented as a quaternion without the z-Rotation. */
  (Quaternionf)(Quaternionf::Identity()) orientation3D, /**< The orientation of the torso represented as a quaternion including the z-Rotation. */
  (Vector2a)(Vector2a::Zero()) angleChange, /**< The change of the angle values. They are saved here, in case a motion frame was dropped. */

  (Matrix3f)(Matrix3f::Zero()) orientation3DCov, /**< The covariance matrix of the 3D torso rotation*/
});
