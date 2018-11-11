/**
 * @file UKFSample.cpp
 *
 * Implementation of Unscented Kalman Filter for robot pose estimation
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Colin Graf
 */

#include "UKFSample.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Modeling/Measurements.h"

using namespace std;

void UKFSample::init(const Pose2f& pose, const Pose2f& defaultPoseDeviation, int id, float validity)
{
  this->id = id;
  this->validity = validity;
  mean << pose.translation.x(), pose.translation.y(), pose.rotation;
  cov = Matrix3f::Zero();
  cov(0, 0) = sqr(defaultPoseDeviation.translation.x());
  cov(1, 1) = sqr(defaultPoseDeviation.translation.y());
  cov(2, 2) = sqr(defaultPoseDeviation.rotation);
  for(int i = 0; i < 7; ++i)
    sigmaPoints[i] = Vector3f::Zero();
}

void UKFSample::mirror()
{
  const Pose2f newPose = Pose2f(pi) + getPose();
  mean.x() = newPose.translation.x();
  mean.y() = newPose.translation.y();
  mean.z() = newPose.rotation;
}

void UKFSample::updateValidity(int frames, float currentValidity)
{
  validity = (validity * (frames - 1) + currentValidity) / frames;
}

void UKFSample::invalidate()
{
  validity = 0.f;
}

float UKFSample::getVarianceWeighting() const
{
  return std::max(cov(0, 0), cov(1, 1)) * cov(2, 2);
}

void UKFSample::twist()
{
  mean.z() += pi;
  mean.z() = Angle::normalize(mean.z());
}

void UKFSample::updateByLandmark(const RegisteredLandmark& landmark)
{
  landmarkSensorUpdate(landmark.w, landmark.p, landmark.cov);
}

void UKFSample::updateByLine(const RegisteredLine& line)
{
  Vector2f orthogonalProjectiona = getOrthogonalProjection(line.pStart, line.pDir, Vector2f::Zero());
  float measuredAngle = -atan2(orthogonalProjectiona.y(), orthogonalProjectiona.x());
  measuredAngle = Angle::normalize(measuredAngle + (line.vertical ? pi_2 : 0));
  float possibleAngle2 = Angle::normalize(measuredAngle - pi);
  if(abs(Angle::normalize(possibleAngle2 - getPose().rotation)) < abs(Angle::normalize(measuredAngle - getPose().rotation)))
    measuredAngle = possibleAngle2;
  float c = cos(measuredAngle), s = sin(measuredAngle);
  Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
  Vector2f orthogonalProjection = angleRotationMatrix * Vector2f(orthogonalProjectiona.x(), orthogonalProjectiona.y());

  Matrix2f cov = line.covCenter;
  cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();
  Covariance::fixCovariance(cov);
  if(line.vertical)
  {
    const float measuredY = line.wStart.y() - orthogonalProjection.y();
    const float yVariance = cov(1, 1);
    const float angleVariance = sqr(atan(sqrt(4.f * yVariance / (line.pStart - line.pEnd).squaredNorm())));
    Matrix2f cov;
    cov << yVariance, 0.f, 0.f, angleVariance;
    lineSensorUpdate(true, Vector2f(measuredY, measuredAngle), cov);
  }
  else
  {
    const float measuredX = line.wStart.x() - orthogonalProjection.x();
    const float xVariance = cov(0, 0);
    const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (line.pStart - line.pEnd).squaredNorm())));
    Matrix2f cov;
    cov << xVariance, 0.f, 0.f, angleVariance;
    lineSensorUpdate(false, Vector2f(measuredX, measuredAngle), cov);
  }
}

void UKFSample::updateByPose(const RegisteredPose& pose, const CameraMatrix& cameraMatrix, const CameraMatrix& inverseCameraMatrix,
                             const Vector2f& currentRotationDeviation, const FieldDimensions& theFieldDimensions)
{
  Vector3f measurement;
  measurement.x() = pose.pose.translation.x();
  measurement.y() = pose.pose.translation.y();
  measurement.z() = pose.pose.rotation;

  const Matrix2f perceivedCenterCovariance = Measurements::positionToCovarianceMatrixInRobotCoordinates(pose.p.translation, 0.f, cameraMatrix, inverseCameraMatrix, currentRotationDeviation);
  const float c = cos(pose.pose.rotation);
  const float s = sin(pose.pose.rotation);
  const Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
  const Matrix2f covXR = angleRotationMatrix * perceivedCenterCovariance * angleRotationMatrix.transpose();
  const Matrix2f circleCov = getCovOfCircle(pose.p.translation, theFieldDimensions.centerCircleRadius, cameraMatrix, inverseCameraMatrix, currentRotationDeviation);
  const Matrix2f covY = angleRotationMatrix * circleCov * angleRotationMatrix.transpose();

  const float xVariance = covXR(0, 0);
  const float yVariance = covY(1, 1);
  const float sqrLineLength = sqr(3 * theFieldDimensions.centerCircleRadius); // TODO: FIX ME
  const float angleVariance = sqr(atan(sqrt(4.f * xVariance / sqrLineLength)));

  Matrix3f cov;
  cov << xVariance, 0.f, 0.f, 0.f, yVariance, 0.f, 0.f, 0.f, angleVariance;
  poseSensorUpdate(measurement, cov);
}

Matrix2f UKFSample::getCovOfCircle(const Vector2f& circlePos, float centerCircleRadius,
                                   const CameraMatrix& cameraMatrix, const CameraMatrix& inverseCameraMatrix,
                                   const Vector2f& currentRotationDeviation) const
{
  float circleDistance = circlePos.norm();
  const float centerCircleDiameter = centerCircleRadius * 2.f;
  Vector2f increasedCirclePos = circlePos;
  if(circleDistance < centerCircleDiameter)
  {
    if(circleDistance < 10.f)
      increasedCirclePos = Vector2f(centerCircleDiameter, 0.f);
    else
      increasedCirclePos *= centerCircleDiameter / circleDistance;
  }
  return Measurements::positionToCovarianceMatrixInRobotCoordinates(increasedCirclePos, 0.f, cameraMatrix, inverseCameraMatrix, currentRotationDeviation);
}

void UKFSample::computeWeightingBasedOnValidity(float baseValidityWeighting)
{
  weighting = baseValidityWeighting + (1.f - baseValidityWeighting) * validity;
}

Vector2f UKFSample::getOrthogonalProjection(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const
{
  const float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  return base + dir * l;
}
