#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Math/UnscentedKalmanFilter.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(InertialDataProvider,
{,
  REQUIRES(CalibrationRequest),
  USES(FootOffset),
  USES(FootSoleRotationCalibration),
  REQUIRES(FootSupport),
  REQUIRES(FsrData),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  USES(GyroOffset),
  REQUIRES(GyroState),
  REQUIRES(InertialSensorData),
  REQUIRES(IMUCalibration),
  USES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  PROVIDES(InertialData),
  LOADS_PARAMETERS(
  {,
    (Vector3a) gyroDeviation, /**< Noise of the gyro in ° / s / sqrt(Hz).Needs to be multiplicated with the square root of the frequency the imu provides new data. */
    (Vector3f) accDeviation, /**< Base acc deviation */
    (Vector3f) accDeviationWhileWalking, /**< Acc deviation while walking */
    (Vector3f) accDeviationWhileStanding, /**< Acc deviation while standing */
    (Vector3f) accDeviationFactor, /**< Acc deviation factor */
    (float) accSoleDeviation, /**< Acc deviation for foot sole generated values. */
    (float) maxMeanSquaredDeviation, /**< Macc measured deviation of multiple acc measurements, to calibrate the gravity vector. */

    // Parameters for the filter stabilization with the foot plane
    (Rangef) feetPressureRatioSagitalRangeXAcc, /**< The ratio between forward and backward FSRs must be inside this range. */
    (Rangef) feetPressureRatioSagitalRangeYAcc, /**< The ratio between forward and backward FSRs must be inside this range. */
    (float) minSumSolePressure, /**< The overall ratio of the sole pressure relative to the max meassured one must be above this ratio. */
    (unsigned) minTimeForLastSupportSwitch, /**< Use x-rotation sole acc generating if last support foot switch happened after this time. */
    (unsigned) maxTimeForLastSupportSwitch, /**< Use x-rotation sole acc generating if last support foot switch happened after this time. */
    (int) maxTimeSinceForwardAndBackwardPressure, /**< Use x-rotation sole acc generating if both feet front and back FSRs measured pressure not to long ago. */
    (float) minGroundContactPointsDistance, /**< Use x-rotation sole acc generating if sole ground contacts are far enough apart. */
    (Angle) soleRotationInterpolation,
    (bool) useIMUAnglesInCalibration,
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
  bool hadXAccUpdate = false;

  Quaternionf soleCalibration[Legs::numOfLegs];
  int soleCalibrationId = -1;

  RingBufferWithSum<float, 50> accelerometerLengths;
  float gravity = Constants::g_1000;

  void update(InertialData& inertialData) override;

  void processAccelerometer(const Vector3f& acc, const bool isAccBasedOnGroundContactPoints, const bool isAccXMeasured);

  void processGyroscope(const Vector3a& gyro);
};
