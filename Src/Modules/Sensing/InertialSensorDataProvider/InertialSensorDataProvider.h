/**
 * @file InertialSensorDataProvider.h
 *
 * This file declares a module that applies the calibration to the measured IMU data and annotates it with covariance matrices
 *
 * @author Yannik Meinken
 */

#pragma once

#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Sensing/InertialSensorData.h"
#include "Framework/Module.h"

MODULE(InertialSensorDataProvider,
{,
  REQUIRES(MotionRobotHealth),
  REQUIRES(RawInertialSensorData),
  REQUIRES(IMUCalibration),
  PROVIDES(InertialSensorData),
  DEFINES_PARAMETERS(
  {,
    (float)(0.000374f) accVariance, /**< White noise variance of the accelerometer measurements (in (m/s²)²). */
    (Angle)(0.00000387f) gyroVariance, /**< White noise variance of the gyro (in (rad / s)²). */
    (Vector3f)({0.2221544f, 0.17215726f, 1.72167881f}) baseAccExtrapolationVariance, /**< Constant part of variance due to extrapolated value instead of new measurement. (In (m/s²)²) */
    (Vector3a)({0.00529817f, 0.01870792f, 0.01661394f}) baseGyroExtrapolationVariance, /**< Constant part of variance due to extrapolated value instead of new measurement. (In (rad / s)²) */
    (Vector3f)({1.65973036f, 1.30990515f, 1.74508419f}) factorAccExtrapolationVariance, /**< Slope of the variance due to extrapolated value. Multiplied with rate of change. (In m/s) */
    (Vector3a)({0.07544756f, 0.1507492f, 0.16514989f}) factorGyroExtrapolationVariance, /**< Slope of the variance due to extrapolated value. Multiplied with rate of change. (In rad) */
    (float)(2.f) overestimateVarianceOfExtrapolationFactor, /**< Factor to scale the variance of extrapolated values. Represents uncertainty's in the model and the violation of independence. */
    (float)(25.f) initialAccChange, /**< Assumed initial change of acceleration, used to estimate the variance in the case we never had a new measurement. Should overestimate the real value. */
    (Angle)(5.f) initialGyroChange, /**< Assumed initial change of rotational velocity, used to estimate the variance in the case we never had a new measurement. Should overestimate the real value. */
  }),
});

class InertialSensorDataProvider : public InertialSensorDataProviderBase
{
private:
  void update(InertialSensorData& inertialSensorData) override;

  Vector3f lastNewRawAccData = Vector3f::Zero(); /**< The last values from the accelerometer that where new */
  Vector3a lastNewRawGyroData = Vector3a::Zero(); /**< The last values from the gyroscope that where new */

  Vector3f accChange = Vector3f::Ones() * initialAccChange; /**< Change between the last two new accelerometer measurements */
  Vector3a gyroChange = Vector3a::Ones() * initialGyroChange; /**< Change between the last two new gyroscope measurements */
};
