/**
 * @file UKFPose2D.cpp
 *
 * Implementation of an Unscented Kalman Filter for robot pose estimation.
 *
 * @author Tim Laue
 * @author Colin Graf
 */

#include "UKFPose2D.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"

void UKFPose2D::motionUpdate(const Pose2f& odometryOffset, const Pose2f& filterProcessDeviation,
                             const Pose2f& odometryDeviation, const Vector2f& odometryRotationDeviation)
{
  generateSigmaPoints();

  // addOdometryToSigmaPoints
  for(int i = 0; i < 7; ++i)
  {
    Vector2f odo(odometryOffset.translation);
    odo.rotate(sigmaPoints[i].z());
    sigmaPoints[i] += Vector3f(odo.x(), odo.y(), odometryOffset.rotation);
  }

  // computeMeanOfSigmaPoints
  mean = sigmaPoints[0];
  for(int i = 1; i < 7; ++i)
    mean += sigmaPoints[i];
  mean *= 1.f / 7.f;

  // computeCovOfSigmaPoints
  const Vector3f d = sigmaPoints[0] - mean;
  cov << d * d.x(), d * d.y(), d * d.z();
  for(int i = 1; i < 7; ++i)
  {
    const Vector3f d = sigmaPoints[i] - mean;
    cov += (Matrix3f() << d * d.x(), d * d.y(), d * d.z()).finished();
  }
  cov *= 0.5f;
  Covariance::fixCovariance(cov);

  // addProcessNoise
  cov(0, 0) += sqr(filterProcessDeviation.translation.x());
  cov(1, 1) += sqr(filterProcessDeviation.translation.y());
  cov(2, 2) += sqr(filterProcessDeviation.rotation);

  Vector2f odo(odometryOffset.translation);
  odo.rotate(mean.z());
  cov(0, 0) += sqr(odo.x() * odometryDeviation.translation.x());
  cov(1, 1) += sqr(odo.y() * odometryDeviation.translation.y());
  cov(2, 2) += sqr(odometryOffset.rotation * odometryDeviation.rotation);
  cov(2, 2) += sqr(odo.x() * odometryRotationDeviation.x());
  cov(2, 2) += sqr(odo.y() * odometryRotationDeviation.y());
  ASSERT(cov(0, 1) == cov(1, 0) && cov(0, 2) == cov(2, 0) && cov(1, 2) == cov(2, 1));
  mean.z() = Angle::normalize(mean.z());
}

void UKFPose2D::generateSigmaPoints()
{
  // Cholesky decomposition
  const float a11 = cov(0, 0);
  const float a21 = (cov(1, 0) + cov(0, 1)) * 0.5f;
  const float a31 = (cov(2, 0) + cov(0, 2)) * 0.5f;
  const float a22 = cov(1, 1);
  const float a32 = (cov(2, 1) + cov(1, 2)) * 0.5f;
  const float a33 = cov(2, 2);

  float& l11(l(0, 0));
  float& l21(l(1, 0));
  float& l31(l(2, 0));
  float& l22(l(1, 1));
  float& l32(l(2, 1));
  float& l33(l(2, 2));

  //ASSERT(a11 >= 0.f);
  l11 = std::sqrt(std::max<>(a11, 0.f));
  if(l11 == 0.f) l11 = 0.0000000001f;
  l21 = a21 / l11;
  l31 = a31 / l11;
  //ASSERT(a22 - l21 * l21 >= 0.f);
  l22 = std::sqrt(std::max<>(a22 - l21 * l21, 0.f));
  if(l22 == 0.f) l22 = 0.0000000001f;
  l32 = (a32 - l31 * l21) / l22;
  //ASSERT(a33 - l31 * l31 - l32 * l32 >= 0.f);
  l33 = std::sqrt(std::max<>(a33 - l31 * l31 - l32 * l32, 0.f));

  sigmaPoints[0] = mean;
  sigmaPoints[1] = mean + l.col(0);
  sigmaPoints[2] = mean - l.col(0);
  sigmaPoints[3] = mean + l.col(1);
  sigmaPoints[4] = mean - l.col(1);
  sigmaPoints[5] = mean + l.col(2);
  sigmaPoints[6] = mean - l.col(2);
}

void UKFPose2D::landmarkSensorUpdate(const Vector2f& landmarkPosition, const Vector2f& reading, const Matrix2f& readingCov)
{
  generateSigmaPoints();

  // computeLandmarkReadings
  Vector2f landmarkReadings[7];
  for(int i = 0; i < 7; ++i)
  {
    Pose2f pose(sigmaPoints[i].z(), sigmaPoints[i].head<2>());
    Vector2f landmarkPosRel = pose.invert() * landmarkPosition; // TODO: optimize this
    landmarkReadings[i] = Vector2f(landmarkPosRel.x(), landmarkPosRel.y());
  }

  // computeMeanOfLandmarkReadings
  Vector2f landmarkReadingMean = landmarkReadings[0];
  for(int i = 1; i < 7; ++i)
    landmarkReadingMean += landmarkReadings[i];
  landmarkReadingMean *= 1.f / 7.f;

  // computeCovOfLandmarkReadingsAndSigmaPoints
  Matrix2x3f landmarkReadingAndMeanCov = Matrix2x3f::Zero();
  for(int i = 0; i < 3; ++i)
  {
    const Vector2f d1 = landmarkReadings[i * 2 + 1] - landmarkReadingMean;
    landmarkReadingAndMeanCov += (Matrix2x3f() << d1 * l(0, i), d1 * l(1, i), d1 * l(2, i)).finished();
    const Vector2f d2 = landmarkReadings[i * 2 + 2] - landmarkReadingMean;
    landmarkReadingAndMeanCov += (Matrix2x3f() << d2 * -l(0, i), d2 * -l(1, i), d2 * -l(2, i)).finished();
  }
  landmarkReadingAndMeanCov *= 0.5f;

  // computeCovOfLandmarkReadingsReadings
  const Vector2f d = landmarkReadings[0] - landmarkReadingMean;
  Matrix2f landmarkReadingCov = Matrix2f::Zero();
  landmarkReadingCov << d * d.x(), d * d.y();
  for(int i = 1; i < 7; ++i)
  {
    const Vector2f d = landmarkReadings[i] - landmarkReadingMean;
    landmarkReadingCov += (Matrix2f() << d * d.x(), d * d.y()).finished();
  }
  landmarkReadingCov *= 0.5f;

  const Matrix3x2f kalmanGain = landmarkReadingAndMeanCov.transpose() * (landmarkReadingCov + readingCov).inverse();
  Vector2f innovation = reading - landmarkReadingMean;
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z() = Angle::normalize(mean.z());
  cov -= kalmanGain * landmarkReadingAndMeanCov;
  Covariance::fixCovariance(cov);
}

void UKFPose2D::lineSensorUpdate(bool vertical, const Vector2f& reading, const Matrix2f& readingCov)
{
  generateSigmaPoints();

  // computeLineReadings
  Vector2f lineReadings[7];
  if(vertical)
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].y(), sigmaPoints[i].z());
  else
    for(int i = 0; i < 7; ++i)
      lineReadings[i] = Vector2f(sigmaPoints[i].x(), sigmaPoints[i].z());

  // computeMeanOfLineReadings
  Vector2f lineReadingMean = lineReadings[0];
  for(int i = 1; i < 7; ++i)
    lineReadingMean += lineReadings[i];
  lineReadingMean *= 1.f / 7.f;

  // computeCovOfLineReadingsAndSigmaPoints
  Matrix2x3f lineReadingAndMeanCov = Matrix2x3f::Zero();
  for(int i = 0; i < 3; ++i)
  {
    const Vector2f d1 = lineReadings[i * 2 + 1] - lineReadingMean;
    lineReadingAndMeanCov += (Matrix2x3f() << d1 * l(0, i), d1 * l(1, i), d1 * l(2, i)).finished();
    const Vector2f d2 = lineReadings[i * 2 + 2] - lineReadingMean;
    lineReadingAndMeanCov += (Matrix2x3f() << d2 * -l(0, i), d2 * -l(1, i), d2 * -l(2, i)).finished();
  }
  lineReadingAndMeanCov *= 0.5f;

  // computeCovOfLineReadingsReadings
  const Vector2f d = lineReadings[0] - lineReadingMean;
  Matrix2f lineReadingCov = (Matrix2f() << d * d.x(), d * d.y()).finished();
  for(int i = 1; i < 7; ++i)
  {
    const Vector2f d = lineReadings[i] - lineReadingMean;
    lineReadingCov += (Matrix2f() << d * d.x(), d * d.y()).finished();
  }
  lineReadingCov *= 0.5f;

  lineReadingMean.y() = Angle::normalize(lineReadingMean.y());
  const Matrix3x2f kalmanGain = lineReadingAndMeanCov.transpose() * (lineReadingCov + readingCov).inverse();
  Vector2f innovation = reading - lineReadingMean;
  innovation.y() = Angle::normalize(innovation.y());
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z() = Angle::normalize(mean.z());
  cov -= kalmanGain * lineReadingAndMeanCov;
  Covariance::fixCovariance(cov);
}

void UKFPose2D::poseSensorUpdate(const Vector3f& reading, const Matrix3f& readingCov)
{
  generateSigmaPoints();

  // computePoseReadings
  Vector3f poseReadings[7];
  for(int i = 0; i < 7; ++i)
    poseReadings[i] = sigmaPoints[i];

  // computeMeanOfPoseReadings
  Vector3f poseReadingMean = poseReadings[0];
  for(int i = 1; i < 7; ++i)
    poseReadingMean += poseReadings[i];
  poseReadingMean *= 1.f / 7.f;

  // computeCovOfPoseReadingsAndSigmaPoints
  Matrix3f poseReadingAndMeanCov = Matrix3f::Zero();
  for(int i = 0; i < 3; ++i)
  {
    const Vector3f d1 = poseReadings[i * 2 + 1] - poseReadingMean;
    poseReadingAndMeanCov += (Matrix3f() << d1 * l(0, i), d1 * l(1, i), d1 * l(2, i)).finished();
    const Vector3f d2 = poseReadings[i * 2 + 2] - poseReadingMean;
    poseReadingAndMeanCov += (Matrix3f() << d2 * -l(0, i), d2 * -l(1, i), d2 * -l(2, i)).finished();
  }
  poseReadingAndMeanCov *= 0.5f;

  // computeCovOfPoseReadingsReadings
  Matrix3f poseReadingCov;
  const Vector3f d = poseReadings[0] - poseReadingMean;
  poseReadingCov << d * d.x(), d * d.y(), d * d.z();
  for(int i = 1; i < 7; ++i)
  {
    const Vector3f d = poseReadings[i] - poseReadingMean;
    poseReadingCov += (Matrix3f() << d * d.x(), d * d.y(), d * d.z()).finished();
  }
  poseReadingCov *= 0.5f;

  poseReadingMean.z() = Angle::normalize(poseReadingMean.z());
  const Matrix3f kalmanGain = poseReadingAndMeanCov.transpose() * (poseReadingCov + readingCov).inverse();
  Vector3f innovation = reading - poseReadingMean;
  innovation.z() = Angle::normalize(innovation.z());
  const Vector3f correction = kalmanGain * innovation;
  mean += correction;
  mean.z() = Angle::normalize(mean.z());
  cov -= kalmanGain * poseReadingAndMeanCov;
  Covariance::fixCovariance(cov);
}
