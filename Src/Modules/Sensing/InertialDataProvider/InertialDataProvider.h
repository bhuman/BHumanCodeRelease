#pragma once

#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/GyroOffset.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(InertialDataProvider,
{,
  USES(GyroOffset),
  USES(MotionInfo),
  REQUIRES(GroundContactState),
  REQUIRES(InertialSensorData),
  REQUIRES(IMUCalibration),
  PROVIDES(InertialData),
  DEFINES_PARAMETERS(
  {,
    (Vector3a)(Vector3a::Constant(0.03_deg)) gyroDeviation, // Noise of the gyro in °/s / sqrt(Hz).
    (Vector3f)(10.f, 10.f, 10.f) accDeviation,
    (Vector3f)(30.f, 30.f, 30.f) accDeviationWhileWalking,
    (Vector3f)(10.f, 10.f, 10.f) accDeviationFactor,
    (Vector3f)(3.f, 3.f, 3.f) accDynamicFilterDeviation,
    (float)(0.1f) maxMeanSquaredDeviation,
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

  UKFM<State> ukf = UKFM<State>(State());

  UKF<3> filteredAccUKF = UKF<3>(Vector3f::Zero());

  Vector2a lastRawAngle = Vector2a::Zero();
  Vector3f lastAccelerometerMeasurement = Vector3f::Zero();
  bool hadAccelerometerMeasurement = false;

  RingBufferWithSum<float, 50> accelerometerLengths;
  float gravity = Constants::g_1000;

  void update(InertialData& inertialData) override;

  void processAccelerometer(const Vector3f& acc);

  void processGyroscope(const Vector3a& gyro);

  void filterAcc(InertialData& id);
};
