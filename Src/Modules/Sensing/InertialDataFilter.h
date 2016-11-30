/**
 * @file InertialDataFilter.h
 * Declaration of module InertialDataFilter.
 * @author Colin Graf
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Module/Module.h"

MODULE(InertialDataFilter,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialSensorData),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  USES(MotionInfo),
  USES(WalkingEngineOutput),
  PROVIDES(InertialData),
  DEFINES_PARAMETERS(
  {,
    (Vector2f)(0.004f, 0.004f) processNoise, /**< The standard deviation of the process. */
    (Vector3f)(1.f, 1.f, 1.f) accNoise, /**< The standard deviation of the inertia sensor. */
    (Vector2a)(20._deg, 30._deg) calculatedAccLimit, /**< Use a calculated angle up to this angle (in rad). (We use the acceleration sensors otherwise.) */
  }),
});

/**
 * @class InertialDataFilter
 * A module for estimating velocity and orientation of the torso.
 */
class InertialDataFilter : public InertialDataFilterBase
{
private:
  /**
   * Represents the state to be estimated.
   */
  class State
  {
  public:
    RotationMatrix rotation; /** The rotation of the torso. */

    /**
     * Adds some world rotation given as angle axis.
     * @param value The flat vector to add.
     * @return A new object after the calculation.
     */
    State operator+(const Vector2f& value) const;

    /**
     * Adds some world rotation given as angle axis.
     * @param value The flat vector to add.
     * @return A reference this object after the calculation.
     */
    State& operator+=(const Vector2f& value);

    /**
     * Calculates a flat difference vector of two states.
     * @return The difference.
     */
    Vector2f operator-(const State& other) const;
  };

  Pose3f lastLeftFoot; /**< The pose of the left foot of the previous iteration. */
  Pose3f lastRightFoot; /**< The pose of the right foot of the previous iteration. */
  unsigned int lastTime = 0; /**< The frame time of the previous iteration. */
  Vector2f safeRawAngle = Vector2f::Zero(); /**< The last not corrupted angle from aldebarans angle estimation algorithm. */

  State mean; /**< The estimate */
  Matrix2f cov = Matrix2f::Zero(); /**< The covariance of the estimate. */
  std::array<State, 5> sigmaPoints; /**< The last calculated sigma points. */

  void update(InertialData& inertialData);

  /**
   * Restores the initial state.
   */
  void reset();

  void dynamicModel(State& state, const Vector3f& rotationOffset);
  void predict(const Vector3f& rotationOffset);

  Vector3f readingModel(const State& sigmaPoint);
  void update(const Vector3f& reading);

  Matrix2f cholOfCov();
  void updateSigmaPoints();
  void meanOfSigmaPoints();
};
