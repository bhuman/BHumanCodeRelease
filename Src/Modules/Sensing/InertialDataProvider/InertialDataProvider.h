#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/InertialData.h"
#include "Math/UnscentedKalmanFilter.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(InertialDataProvider,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(GroundContactState),
  USES(GyroOffset),
  REQUIRES(InertialSensorData),
  REQUIRES(IMUCalibration),
  USES(MotionInfo),
  PROVIDES(InertialData),
  LOADS_PARAMETERS(
  {,
    (Vector3a) gyroVariance, /**< Noise of the gyro in (° / s)² / sqrt(Hz).Needs to be multiplicated with the square root of the frequency the imu provides new data. */
    (Vector3f) accVariance, /**< Base acc variance */
    (Vector3f) accVarianceWhileWalking, /**< Acc variance while walking */
    (Vector3f) accVarianceWhileStanding, /**< Acc variance while standing */
    (Vector3f) accVarianceFactor, /**< Acc variance factor */
    (float) maxMeanSquaredDeviation, /**< Macc measured variance of multiple acc measurements, to calibrate the gravity vector. */
  }),
});

class InertialDataProvider : public InertialDataProviderBase
{
private:
  struct State : public Manifold<3>
  {
    Quaternionf orientation;

    State(const Quaternionf& orientation = Quaternionf::Identity()) : orientation(orientation)
    {
      // orientation = Quaternionf::Identity(); // Workaround, usual initialization crashes in Debug
    }

    State operator+(const Vectorf& angleAxis) const;
    State& operator+=(const Vectorf& angleAxis);
    Vectorf operator-(const State& other) const;
  };

  RingBufferWithSum<float, 10> deviation10;
  RingBufferWithSum<float, 50> deviation50;

  UKFM<State> ukf = UKFM<State>(State());

  Vector2a lastRawAngle = Vector2a::Zero();
  Vector3f lastAccelerometerMeasurement = Vector3f::Zero();
  bool hadAccelerometerMeasurement = false;

  RingBufferWithSum<float, 50> accelerometerLengths;
  float gravity = Constants::g_1000;

  void update(InertialData& inertialData) override;

  void processAccelerometer(const Vector3f& acc);

  void processGyroscope(const Vector3a& gyro);
};
