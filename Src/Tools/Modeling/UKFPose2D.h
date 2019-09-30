/**
 * @file UKFPose2D.h
 *
 * Declaration of an Unscented Kalman Filter for robot pose estimation.
 *
 * @author Tim Laue
 * @author Colin Graf
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"

/**
 * @class UKFPose2D
 *
 * Hypothesis of a robot's pose in 2D, modeled as an Unscented Kalman Filter.
 * This class is not intended to be used directly but as a base class for
 * specific localization implementations.
 */
class UKFPose2D
{
protected:
  Vector3f mean = Vector3f::Zero();   /**< The estimated pose in 2D. */
  Matrix3f cov = Matrix3f::Zero();    /**< The covariance matrix of the estimate. */
  Vector3f sigmaPoints[7];            /**< Sigma points for updating the filter. */
  Matrix3f l = Matrix3f::Zero();      /**< Last computed Cholesky decomposition. */

public:
  /** Returns the internal representation of the state's mean as the commonly used Pose2f type
   * @return The pose
   */
  Pose2f getPose() const
  {
    return Pose2f(mean.z(), mean.head<2>());
  }

  /** Returns a reference to the internal representation of the state's covariance matrix
   * @return A reference to the matrix
   */
  const Matrix3f& getCov() const
  {
    return cov;
  }

  void motionUpdate(const Pose2f& odometryOffset, const Pose2f& filterProcessDeviation,
                    const Pose2f& odometryDeviation, const Vector2f& odometryRotationDeviation);

protected:
  /** Computes the 7 sigma points (in the sigmaPoints array) based on the current content of cov */
  void generateSigmaPoints();

  void landmarkSensorUpdate(const Vector2f& landmarkPosition, const Vector2f& reading, const Matrix2f& readingCov);

  void lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2f& readingCov);

  void poseSensorUpdate(const Vector3f& reading, const Matrix3f& readingCov);
};
