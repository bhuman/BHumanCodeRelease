#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"

class TorsoAngleEstimator
{
public:
  TorsoAngleEstimator(const InertialSensorData& inertialSensorData, const RobotModel& robotModel,
                      const MotionRequest& motionRequest) :
    theInertialSensorData(inertialSensorData), theRobotModel(robotModel), theMotionRequest(motionRequest)
  {}

  Vector2a update(float walkSpeedX);
  void reset();

private:
  class State
  {
  public:
    RotationMatrix rotation; /** The rotation of the torso. */

    State operator+(const Vector2f& value) const;
    State& operator+=(const Vector2f& value);
    Vector2f operator-(const State& other) const;
  };

  const InertialSensorData& theInertialSensorData;
  const RobotModel& theRobotModel;
  const MotionRequest& theMotionRequest;

  Pose3f lastLeftFoot; /**< The pose of the left foot of the previous iteration. */
  Pose3f lastRightFoot; /**< The pose of the right foot of the previous iteration. */

  State mean; /**< The estimate */
  Matrix2f cov = Matrix2f::Zero(); /**< The covariance of the estimate. */
  std::array<State, 5> sigmaPoints; /**< The last calculated sigma points. */

  Vector2f processNoise = Vector2f(0.004f, 0.004f); /**< The standard deviation of the process. */
  Vector3f accNoise = Vector3f(1.f, 1.f, 1.f); /**< The standard deviation of the inertia sensor. */
  Vector2a calculatedAccLimit = Vector2a(20._deg, 30._deg); /**< Use a calculated angle up to this angle (in rad). (We use the acceleration sensors otherwise.) */

  void dynamicModel(State& state, const Vector3f& rotationOffset);
  void predict(const Vector3f& rotationOffset);

  Vector3f readingModel(const State& sigmaPoint);
  void update(const Vector3f& reading);

  Matrix2f cholOfCov();
  void updateSigmaPoints();
  void meanOfSigmaPoints();
};
