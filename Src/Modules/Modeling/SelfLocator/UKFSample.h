/**
 * @file UKFSample.h
 *
 * Declaration of Unscented Kalman Filter for robot pose estimation
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Colin Graf
 */

#pragma once

#include "PerceptRegistration.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Modeling/UKFPose2D.h"

/**
 * @class UKFSample
 *
 * Hypothesis of a robot's pose, modeled as an Unscented Kalman Filter
 */
class UKFSample : public UKFPose2D
{
public:
  float weighting;
  float validity;
  int id;

  void mirror();

  void twist();

  void updateValidity(int frames, float currentValidity);

  void invalidate();

  void computeWeightingBasedOnValidity(float baseValidityWeighting);

  float getVarianceWeighting() const;

  void init(const Pose2f& pose, const Pose2f& defaultPoseDeviation, int number, float validity);

  void updateByLandmark(const RegisteredLandmark& landmark);

  void updateByLine(const RegisteredLine& line);

  void updateByPose(const RegisteredPose& pose, const CameraMatrix& cameraMatrix, const CameraMatrix& inverseCameraMatrix,
                    const Vector2f& currentRotationDeviation, const FieldDimensions& theFieldDimensions);

private:
  Matrix2f getCovOfCircle(const Vector2f& circlePos, float centerCircleRadius,
                          const CameraMatrix& cameraMatrix, const CameraMatrix& inverseCameraMatrix,
                          const Vector2f& currentRotationDeviation) const;

  Vector2f getOrthogonalProjection(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const;
};
