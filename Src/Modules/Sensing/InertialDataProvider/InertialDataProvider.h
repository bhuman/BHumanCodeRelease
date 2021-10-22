#pragma once

#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(InertialDataProvider,
{,
  USES(GyroOffset),
  USES(MotionInfo),
  USES(FootOffset),
  REQUIRES(FootSupport),
  REQUIRES(FootSoleRotationCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(GyroState),
  REQUIRES(InertialSensorData),
  REQUIRES(IMUCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  PROVIDES(InertialData),
  LOADS_PARAMETERS(
  {,
    (Vector3a) gyroDeviation, // Noise of the gyro in °/s / sqrt(Hz). Needs to be multiplicated with the square root of the frequency the imu provides new data.
    (Vector3f) accDeviation,
    (Vector3f) accDeviationWhileWalking,
    (Vector3f) accDeviationWhileStanding,
    (Vector3f) accDeviationFactor,
    (float) maxMeanSquaredDeviation,

    // Parameters for the filter stabilization with the foot plane
    (int) maxTimeSinceForwardAndBackwardPressure,
    (unsigned) minTimeForLastSupportSwitch,
    (unsigned) maxTimeForLastSupportSwitch,
    (float) minGroundContactPointsDistance,
    (Angle) maxAllowedFeetPlaneXRotation,
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
  Vector3f lastAccelerometerMeasurement = Vector3f::Zero();
  bool hadAccelerometerMeasurement = false;

  RingBufferWithSum<float, 50> accelerometerLengths;
  float gravity = Constants::g_1000;
  bool hadXAccUpdate = false;

  void update(InertialData& inertialData) override;

  void processAccelerometer(const Vector3f& acc, const bool isAccBasedOnGroundContactPoints, const bool isAccXMeasured);

  void processGyroscope(const Vector3a& gyro);
};
