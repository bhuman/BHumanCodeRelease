/**
* @file InertiaSensorFilter.h
* Declaration of module InertiaSensorFilter.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Matrix.h"

MODULE(InertiaSensorFilter)
  REQUIRES(FrameInfo)
  REQUIRES(InertiaSensorData)
  REQUIRES(RobotModel)
  REQUIRES(SensorData)
  REQUIRES(RobotDimensions)
  USES(MotionInfo)
  USES(WalkingEngineOutput)
  PROVIDES_WITH_MODIFY(OrientationData)
  DEFINES_PARAMETER(Vector2f, processNoise, Vector2f(0.004f, 0.004f)) /**< The standard deviation of the process. */
  DEFINES_PARAMETER(Vector3f, accNoise, Vector3f(1.f, 1.f, 1.f)) /**< The standard deviation of the inertia sensor. */
  DEFINES_PARAMETER(Vector2f, calculatedAccLimit, Vector2f(fromDegrees(20.f), fromDegrees(30.f))) /**< Use a calculated angle up to this angle (in rad). (We use the acceleration sensors otherwise.) */
END_MODULE

/**
* @class InertiaSensorFilter
* A module for estimating velocity and orientation of the torso.
*/
class InertiaSensorFilter : public InertiaSensorFilterBase
{
public:
  /** Default constructor. */
  InertiaSensorFilter();

private:
  /**
  * Represents the state to be estimated.
  */
  class State
  {
  public:
    RotationMatrix rotation; /** The rotation of the torso. */

    /** Default constructor. */
    State() {}

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

  Matrix2x2f processCov; /**< The covariance matrix for process noise. */
  Matrix3x3f sensorCov; /**< The covariance matrix for sensor noise. */

  Pose3D lastLeftFoot; /**< The pose of the left foot of the previous iteration. */
  Pose3D lastRightFoot; /**< The pose of the right foot of the previous iteration. */
  unsigned int lastTime; /**< The frame time of the previous iteration. */
  Vector2f safeRawAngle; /**< The last not corrupted angle from aldebarans angle estimation algorithm. */

  State x; /**< The estimate */
  Matrix2x2f cov; /**< The covariance of the estimate. */
  Matrix2x2f l; /**< The last caculated cholesky decomposition of \c cov. */
  State sigmaPoints[5]; /**< The last calculated sigma points. */

  Vector3f sigmaReadings[5]; /**< The expected sensor values at the sigma points. */
  Vector3f readingMean; /**< The mean of the expected sensor values which was determined by using the sigma velocities. */
  Matrix3x3f readingsCov;
  Matrix3x2f readingsSigmaPointsCov;

  /**
  * Updates the OrientationData representation.
  * @param orientationData The orientation data representation which is updated by this module.
  */
  void update(OrientationData& orientationData);

  /**
   * Calculates parameter dependent constants used to speed up some calculations.
   */
  void calculateConstants();

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
  void covOfSigmaReadingsAndSigmaPoints();
  void covOfSigmaReadings();

  inline Matrix2x2f tensor(const Vector2f& a, const Vector2f& b) const
  {
    return Matrix2x2f(a * b.x, a * b.y);
  }

  inline Matrix2x2f tensor(const Vector2f& a) const
  {
    return Matrix2x2f(a * a.x, a * a.y);
  }

  inline Matrix3x2f tensor(const Vector3f& a, const Vector2f& b)
  {
    return Matrix3x2f(a * b.x, a * b.y);
  }

  inline Matrix3x3f tensor(const Vector3f& a)
  {
    return Matrix3x3f(a * a.x, a * a.y, a * a.z);
  }
};
