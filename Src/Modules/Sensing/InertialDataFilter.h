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
  Matrix2f l = Matrix2f::Zero(); /**< The last caculated cholesky decomposition of \c cov. */
  State sigmaPoints[5]; /**< The last calculated sigma points. */

  Vector3f sigmaReadings[5]; /**< The expected sensor values at the sigma points. */
  Vector3f readingMean = Vector3f::Zero(); /**< The mean of the expected sensor values which was determined by using the sigma velocities. */

  void update(InertialData& inertialData);

  /**
   * Restores the initial state.
   */
  void reset();

  void predict(const RotationMatrix& rotationOffset);
  void readingUpdate(const Vector3f& reading);

  void cholOfCov();
  void generateSigmaPoints();
  void meanOfSigmaPoints();
  void covOfSigmaPoints();

  void readingModel(const State& sigmaPoint, Vector3f& reading);
  void meanOfSigmaReadings();
  Matrix3x2f covOfSigmaReadingsAndSigmaPoints();
  Matrix3f covOfSigmaReadings();

  Matrix2f tensor(const Vector2f& a) const
  {
    return (Matrix2f() << a * a.x(), a * a.y()).finished();
  }

  Matrix3x2f tensor(const Vector3f& a, const Vector2f& b)
  {
    return (Matrix3x2f() << a * b.x(), a * b.y()).finished();
  }

  Matrix3f tensor(const Vector3f& a) const
  {
    return (Matrix3f() << a * a.x(), a * a.y(), a * a.z()).finished();
  }
};
