/**
* @file UKFSample.h
*
* Declaration of Unscented Kalman Filter for robot pose estimation
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Colin Graf
*/

#pragma once

#include "SelfLocatorBase.h"
#include "Tools/Math/Matrix.h"
#include "Tools/RingBufferWithSum.h"
#include "FieldModel.h"

/**
* @class UKFSample
*
* Hypothesis of a robot's pose, modeled as an Unscented Kalman Filter
*/
class UKFSample
{
public:
  Vector3f   mean;          /**< The estimated pose. */

private:
  Matrix3x3f cov;           /**< The covariance matrix of the estimate. */
  Vector3f sigmaPoints[7];  /**< Sigma points for updating the filter */
  Matrix3x3f l;             /**< Last computed cholesky decomposition */
  RingBufferWithSum<float,60> validityBuffer;

public:

  float weighting;

  float validity;
  
  int number;

  Pose2D getPose() const;

  Matrix3x3f& getCov() { return cov; };

  void mirror();

  float computeValidity(const FieldDimensions& fieldDimensions);

  void invalidate();

  void twist();

  void computeWeightingBasedOnValidity(const FieldDimensions& fieldDimensions, const SelfLocatorBase::Parameters& parameters);

  float getVarianceWeighting() const;

  void init(const Pose2D& pose, const SelfLocatorBase::Parameters& parameters, int number);

  void motionUpdate(const Pose2D& odometryOffset, const SelfLocatorBase::Parameters& parameters);

  void performOdometryUpdate(const Pose2D& odometryOffset, const SelfLocatorBase::Parameters& parameters);

  void updateByGoalPercept(const GoalPercept& goalPercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                           const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

  void updateByLinePercept(const LinePercept& linePercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                           const FieldDimensions& fieldDimensions, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

  void draw(bool simple = false);

private:
  Matrix2x2f getCovOfPointInWorld(const Vector2<>& pointInWorld, float pointZInWorld,
    const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix, const SelfLocatorBase::Parameters& parameters) const;

  Matrix2x2f getCovOfCircle(const Vector2<>& circlePos, float centerCircleRadius, const MotionInfo& motionInfo,
    const CameraMatrix& cameraMatrix, const SelfLocatorBase::Parameters& parameters) const;

  void landmarkSensorUpdate(const Vector2<>& landmarkPosition, const Vector2f& reading, const Matrix2x2f& readingCov);

  void lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2x2f& readingCov);

  void poseSensorUpdate(const Vector3f& reading, const Matrix3x3f& readingCov);

  void generateSigmaPoints();

  Vector2<> getOrthogonalProjection(const Vector2<>& base, const Vector2<>& dir, const Vector2<>& point) const;

  float computeAngleWeighting(float measuredAngle, const Vector2<>& modelPosition,
                              const Pose2D& robotPose, float standardDeviation) const;

  float computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2<>& modelPosition,
                              const Pose2D& robotPose, float cameraZ, float standardDeviation) const;
};
