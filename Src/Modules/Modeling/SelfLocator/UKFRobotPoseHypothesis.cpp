/**
 * @file UKFRobotPoseHypothesis.cpp
 *
 * Implementation of a robot pose estimate based on an Unscented Kalman Filter
 *
 * @author Tim Laue
 * @author Colin Graf
 */

#include "UKFRobotPoseHypothesis.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Measurements.h"

using namespace std;

void UKFRobotPoseHypothesis::init(const Pose2f& pose, const Pose2f& poseDeviation, int id, float validity)
{
  this->id = id;
  this->validity = validity;
  mean << pose.translation.x(), pose.translation.y(), pose.rotation;
  cov = Matrix3f::Zero();
  cov(0, 0) = sqr(poseDeviation.translation.x());
  cov(1, 1) = sqr(poseDeviation.translation.y());
  cov(2, 2) = sqr(poseDeviation.rotation);
  for(int i = 0; i < 7; ++i)
    sigmaPoints[i] = Vector3f::Zero();
}

void UKFRobotPoseHypothesis::mirror()
{
  const Pose2f newPose = Pose2f(pi) + getPose();
  mean.x() = newPose.translation.x();
  mean.y() = newPose.translation.y();
  mean.z() = newPose.rotation;
}

void UKFRobotPoseHypothesis::updateValidity(int frames, float currentValidity)
{
  validity = (validity * (frames - 1) + currentValidity) / frames;
}

void UKFRobotPoseHypothesis::computeWeightingBasedOnValidity(float baseValidityWeighting)
{
  weighting = baseValidityWeighting + (1.f - baseValidityWeighting) * validity;
}

void UKFRobotPoseHypothesis::invalidate()
{
  validity = 0.f;
}

float UKFRobotPoseHypothesis::getCombinedVariance() const
{
  return std::max(cov(0, 0), cov(1, 1)) * cov(2, 2);
}

void UKFRobotPoseHypothesis::twist()
{
  mean.z() += pi;
  mean.z() = Angle::normalize(mean.z());
}

void UKFRobotPoseHypothesis::updateByLandmark(const RegisteredLandmark& landmark)
{
  landmarkSensorUpdate(landmark.model, landmark.percept, landmark.covPercept);
}

void UKFRobotPoseHypothesis::updateByLineOnCenterCircle(const RegisteredLine& line, float centerCircleRadius)
{
  ASSERT(line.partOfCenterCircle);
  // Create a fake landmark update by computing the orthogonal on the line center.
  // By scaling the orthogonal to the center circle radius (in the right direction!),
  // we can compute a position close to the center circle. This is our landmark!
  const Vector2f lineCenter = (line.perceptStart + line.perceptEnd) * 0.5f;
  // Compute both possible positions in field coordinates (we do not know, in which direction the actual center circle is):
  Vector2f orthogonalA = line.perceptDirection;
  orthogonalA.normalize(centerCircleRadius);
  Vector2f orthogonalB = orthogonalA;
  orthogonalA.rotateRight();
  orthogonalB.rotateLeft();
  const Pose2f pose = getPose();
  const Vector2f pointA = pose * (lineCenter + orthogonalA);
  const Vector2f pointB = pose * (lineCenter + orthogonalB);
  // The point that is closer to the field center (0,0) is used to
  // create a fake measurement in coordinates relative to the robot.
  Vector2f fakeMeasurement;
  if(pointA.norm() < pointB.norm())
    fakeMeasurement = Transformation::fieldToRobot(pose, pointA);
  else
    fakeMeasurement = Transformation::fieldToRobot(pose, pointB);
  // Pretend to have measured the center circle:
  landmarkSensorUpdate(Vector2f(0.f,0.f), fakeMeasurement, line.covPerceptCenter);
}

void UKFRobotPoseHypothesis::updateByLine(const RegisteredLine& line)
{
  ASSERT(line.partOfCenterCircle == false);
  Vector2f orthogonalProjectiona = Geometry::getOrthogonalProjectionOfPointOnLine(line.perceptStart, line.perceptDirection, Vector2f::Zero());
  float measuredAngle = -atan2(orthogonalProjectiona.y(), orthogonalProjectiona.x());
  measuredAngle = Angle::normalize(measuredAngle + (line.parallelToWorldModelXAxis ? pi_2 : 0));
  float possibleAngle2 = Angle::normalize(measuredAngle - pi);
  if(abs(Angle::normalize(possibleAngle2 - getPose().rotation)) < abs(Angle::normalize(measuredAngle - getPose().rotation)))
    measuredAngle = possibleAngle2;
  float c = cos(measuredAngle), s = sin(measuredAngle);
  Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
  Vector2f orthogonalProjection = angleRotationMatrix * Vector2f(orthogonalProjectiona.x(), orthogonalProjectiona.y());

  Matrix2f cov = line.covPerceptCenter;
  cov = angleRotationMatrix * cov * angleRotationMatrix.transpose();
  Covariance::fixCovariance(cov);
  if(line.parallelToWorldModelXAxis)
  {
    const float measuredY = line.modelStart.y() - orthogonalProjection.y();
    const float yVariance = cov(1, 1);
    const float angleVariance = sqr(atan(sqrt(4.f * yVariance / (line.perceptStart - line.perceptEnd).squaredNorm())));
    Matrix2f cov;
    cov << yVariance, 0.f, 0.f, angleVariance;
    lineSensorUpdate(true, Vector2f(measuredY, measuredAngle), cov);
  }
  else
  {
    const float measuredX = line.modelStart.x() - orthogonalProjection.x();
    const float xVariance = cov(0, 0);
    const float angleVariance = sqr(atan(sqrt(4.f * xVariance / (line.perceptStart - line.perceptEnd).squaredNorm())));
    Matrix2f cov;
    cov << xVariance, 0.f, 0.f, angleVariance;
    lineSensorUpdate(false, Vector2f(measuredX, measuredAngle), cov);
  }
}

void UKFRobotPoseHypothesis::updateByPose(const RegisteredAbsolutePoseMeasurement& pose, const CameraMatrix& cameraMatrix, const CameraMatrix& inverseCameraMatrix,
                                          const Vector2f& currentRotationDeviation, const FieldDimensions& theFieldDimensions)
{
  Vector3f measurement;
  measurement.x() = pose.absolutePoseOnField.translation.x();
  measurement.y() = pose.absolutePoseOnField.translation.y();
  measurement.z() = pose.absolutePoseOnField.rotation;

  const Matrix2f perceivedCenterCovariance = Measurements::positionToCovarianceMatrixInRobotCoordinates(pose.perceivedRelativePose.translation, 0.f, cameraMatrix, inverseCameraMatrix, currentRotationDeviation);
  const float c = cos(pose.absolutePoseOnField.rotation);
  const float s = sin(pose.absolutePoseOnField.rotation);
  const Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
  const Matrix2f covXR = angleRotationMatrix * perceivedCenterCovariance * angleRotationMatrix.transpose();
  const Matrix2f correctedCov = computeCorrectedPoseCovariance(pose.perceivedRelativePose.translation, theFieldDimensions.centerCircleRadius * 0.7834f, cameraMatrix, inverseCameraMatrix, currentRotationDeviation);
  const Matrix2f covY = angleRotationMatrix * correctedCov * angleRotationMatrix.transpose();

  const float xVariance = covXR(0, 0);
  const float yVariance = covY(1, 1);
  const float sqrLineLength = sqr(3 * theFieldDimensions.centerCircleRadius); // TODO: FIX ME
  const float angleVariance = sqr(atan(sqrt(4.f * xVariance / sqrLineLength)));

  Matrix3f cov;
  cov << xVariance, 0.f, 0.f, 0.f, yVariance, 0.f, 0.f, 0.f, angleVariance;
  poseSensorUpdate(measurement, cov);
}

Matrix2f UKFRobotPoseHypothesis::computeCorrectedPoseCovariance(const Vector2f& perceivedPosition, float minimumDistance,
                                                                const CameraMatrix& cameraMatrix, const CameraMatrix& inverseCameraMatrix,
                                                                const Vector2f& currentRotationDeviation) const
{
  const float perceivedDistance = perceivedPosition.norm();
  return Measurements::positionToCovarianceMatrixInRobotCoordinates(perceivedDistance < minimumDistance ? Vector2f(minimumDistance, 0.f) : perceivedPosition,
                                                                    0.f, cameraMatrix, inverseCameraMatrix, currentRotationDeviation);
}
