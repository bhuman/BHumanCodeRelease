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
#include "Tools/Math/Eigen.h"
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
  Vector3f mean = Vector3f::Zero();   /**< The estimated pose. */

private:
  Matrix3f cov = Matrix3f::Zero(); /**< The covariance matrix of the estimate. */
  Vector3f sigmaPoints[7];                    /**< Sigma points for updating the filter */
  Matrix3f l = Matrix3f::Zero();   /**< Last computed cholesky decomposition */
  RingBufferWithSum<float, 60> validityBuffer;

public:

  float weighting;

  float validity;

  int id;

  Pose2f getPose() const;

  Matrix3f& getCov() { return cov; };

  void mirror();

  float computeValidity(const FieldDimensions& fieldDimensions);

  void invalidate();

  void twist();

  void computeWeightingBasedOnValidity(const FieldDimensions& fieldDimensions, const SelfLocatorBase::Parameters& parameters);

  float getVarianceWeighting() const;

  void init(const Pose2f& pose, const SelfLocatorBase::Parameters& parameters, int number);

  void motionUpdate(const Pose2f& odometryOffset, const SelfLocatorBase::Parameters& parameters);

  void performOdometryUpdate(const Pose2f& odometryOffset, const SelfLocatorBase::Parameters& parameters);

  void updateByGoalPercept(const GoalPercept& goalPercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                           const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);
  
  void updateByPenaltyMarkPercept(const PenaltyMarkPercept& penaltyMarkPercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                                  const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

  void updateByLinePercept(const LinePercept& linePercept, const FieldModel& fieldModel, const SelfLocatorBase::Parameters& parameters,
                           const FieldDimensions& fieldDimensions, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

  void draw(bool simple = false);

private:
  Matrix2f getCovOfPointInWorld(const Vector2f& pointInWorld, float pointZInWorld,
    const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix, const SelfLocatorBase::Parameters& parameters) const;

  Matrix2f getCovOfCircle(const Vector2f& circlePos, float centerCircleRadius, const MotionInfo& motionInfo,
    const CameraMatrix& cameraMatrix, const SelfLocatorBase::Parameters& parameters) const;

  void landmarkSensorUpdate(const Vector2f& landmarkPosition, const Vector2f& reading, const Matrix2f& readingCov);

  void lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2f& readingCov);

  void poseSensorUpdate(const Vector3f& reading, const Matrix3f& readingCov);

  void generateSigmaPoints();

  Vector2f getOrthogonalProjection(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const;

  float computeAngleWeighting(float measuredAngle, const Vector2f& modelPosition,
                              const Pose2f& robotPose, float standardDeviation) const;

  float computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2f& modelPosition,
                              const Pose2f& robotPose, float cameraZ, float standardDeviation) const;
};
