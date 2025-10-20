/**
 * @file UKFPose2D.h
 *
 * Declaration of an Unscented Kalman Filter for robot pose estimation.
 *
 * @author Tim Laue
 * @author Colin Graf
 */

#pragma once

#include "Math/Eigen.h"
#include "Math/Pose2f.h"

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

  /** Pose update based on the assumed robot motion
   * @param odometryOffset The pos(e)itional changes regarding translation and rotation (as reported by motion modules) | cspell:disable-line
   * @param filterProcessDeviation Process noise for Kalman filter update
   * @param odometryDeviation The assumed uncertainty in odometry information
   * @param odometryRotationDeviation Additional odometry uncertainty of rotation that affects translation
   */
  void motionUpdate(const Pose2f& odometryOffset, const Pose2f& filterProcessDeviation,
                    const Pose2f& odometryDeviation, const Vector2f& odometryRotationDeviation);

protected:
  /** Computes the 7 sigma points (in the sigmaPoints array), based on the current content of cov */
  void generateSigmaPoints();

  /** Pose update based on the measurement of a perceived landmark
   * @param landmarkPosition The model position of the landmark (in absolute field coordinates)
   * @param reading The position of the measured landmark (in coordinates relative to the robot)
   * @param readingCov The covariance of the measurement
   */
  void landmarkSensorUpdate(const Vector2f& landmarkPosition, const Vector2f& reading, const Matrix2f& readingCov);

  /** Pose update based on the measurement of a perceived line
   * @param lineIsParallelToWorldModelXAxis Flag. Set this to "true", if the perceived line is assumed to be parallel to the field's x-axis (given a previous registration process).
   *                 In this case, only the robot's y-coordinate as well as its rotation can be updated. If this flag is set to false,
   *                 only the robot's x-coordinate as well as its rotation can be updated, as the line is assumed to be parallel the field's halfway line (y-axis).
   *                 We assume that there are no major diagonal lines on a football pitch.
   * @param reading The first element of this vector is a measured coordinate on the field.
   *                If lineIsParallelToWorldModelXAxis is set to true, it is a y-coordinate, an x-coordinate otherwise. The second component is always a measured rotation.
   * @param readingCov The covariance of the measurement/reading
   */
  void lineSensorUpdate(bool lineIsParallelToWorldModelXAxis, const Vector2f& reading, const Matrix2f& readingCov);

  /** Pose update based on the (somehow virtual) absolute measurement of the own pose
   * @param reading The measured pose (in absolute field coordinates)
   * @param readingCov The covariance of the measurement (based on the relative measurement of some features indicating the absolute pose)
   */
  void poseSensorUpdate(const Vector3f& reading, const Matrix3f& readingCov);
};
