#pragma once

#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Module/Module.h"

MODULE(InertialDataProvider,
{,
  USES(MotionInfo),
  REQUIRES(InertialSensorData),
  REQUIRES(IMUCalibration),
  PROVIDES(InertialData),
  DEFINES_PARAMETERS(
  {,
    (Vector3a)(Vector3a::Constant(0.03_deg)) gyroDeviation, // Noise of the gyro in °/s / sqrt(Hz).
    (Vector3f)(10.f, 10.f, 10.f) accDeviation,
    (Vector3f)(50.f, 50.f, 50.f) accDeviationWhileWalking,
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

  Vector2a lastRawAngle = Vector2a::Zero();

  void update(InertialData& inertialData);

  void estimate(const Vector3a& gyro, const Vector3f& acc, float timePassed, const Vector3a& gyroDeviation, const Vector3f& accDeviation);
};
