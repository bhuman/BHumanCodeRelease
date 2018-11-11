#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * Encapsulates the IMU sensor data as it is provided by NAOqi.
 * The provided data is never set to SensorData::off! At most it is zero initialized.
 */
STREAMABLE(InertialSensorData,
{
  void draw(),

  (Vector3a)(Vector3a::Zero()) gyro, /**< The change in orientation around the x-, y-, and z-axis (in radian/s). */
  (Vector3f)(Vector3f::Zero()) acc, /**< The acceleration along the x-, y- and z-axis (in m/s^2). */
  (Vector3a)(Vector3a::Zero()) angle, /**< The orientation of the torso (in rad). */
});
