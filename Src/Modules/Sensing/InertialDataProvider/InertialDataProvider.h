#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Sensing/InertialData.h"
#include "Math/UnscentedKalmanFilter.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(InertialDataProvider,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(GroundContactState),
  REQUIRES(GyroOffset),
  REQUIRES(IMUValueState),
  REQUIRES(InertialSensorData),
  USES(MotionInfo),
  REQUIRES(MotionRobotHealth),
  PROVIDES(InertialData),
  LOADS_PARAMETERS(
  {,
    (Vector3a) gyroVariance, /**< Noise of the gyro in (° / s)² / sqrt(Hz). Needs to be multiplied with the square root of the frequency the imu provides new data. */
    (Vector3f) accVariance, /**< Base acc variance */
    (Vector3f) accVarianceWhileWalking, /**< Acc variance while walking */
    (Vector3f) accVarianceWhileStanding, /**< Acc variance while standing */
    (Vector3f) accVarianceFactor, /**< Acc variance factor */
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

  void update(InertialData& inertialData) override;

  void processAccelerometer(const Vector3f& acc);

  void processGyroscope(const Vector3a& gyro);

  unsigned framesSinceStart = 0;
};
