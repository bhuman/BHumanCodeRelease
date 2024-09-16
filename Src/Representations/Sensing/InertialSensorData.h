/**
 * @file InertialSensorData.h
 * This representation contains the calibrated IMU sensor data with additional meta data.
 * @author Yannik Meinken
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct InertialSensorData
 * Encapsulates the calibrated IMU sensor data. Together with its covariance.
 */
STREAMABLE_WITH_BASE(InertialSensorData, RawInertialSensorData,
{,
  (Eigen::Matrix<Angle, 3, 3>)(Eigen::Matrix<Angle, 3, 3>::Zero()) gyroCovariance, /**< The covariance of the gyro measurements (in radian/s)². */
  (Matrix3f)(Matrix3f::Zero()) accCovariance, /**< The covariance of the acceleration measurements (in m/s^2)². */

  (bool)(false) newAccData, /**< True if new data from the accelerometer was received. */
  (bool)(false) newGyroData, /**< True if new data from the gyroscope was received. */
});
