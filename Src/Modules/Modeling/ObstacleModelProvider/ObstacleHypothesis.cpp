/**.
 * @file ObstacleHypothesis.cpp
 *
 * Implementation of a class that represents a possible obstacle.
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 */

#include "ObstacleHypothesis.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"

#include <algorithm>

ObstacleHypothesis::ObstacleHypothesis(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right,
                                       const unsigned lastSeen, const Type type, const unsigned seenCount)
  : seenCount(seenCount)
{
  ASSERT(covariance(1, 0) == covariance(0, 1));
  this->covariance = covariance;
  this->center = center;
  this->left = left;
  this->right = right;
  this->velocity.setZero();
  this->lastSeen = lastSeen;
  this->type = type;

  team = isTeammate() ? -1 : (isOpponent() ? 1 : 0);
  upright = type < Obstacle::fallenSomeRobot ? 1 : -1;
}

ObstacleHypothesis::ObstacleHypothesis(const Type type)
{
  this->type = type;

  team = isTeammate() ? -1 : (isOpponent() ? 1 : 0);
  upright = type < Obstacle::fallenSomeRobot ? 1 : -1;
}

bool ObstacleHypothesis::isBehind(const ObstacleHypothesis& other) const
{
  const bool allPointsFurtherAway = left.squaredNorm() > other.left.squaredNorm() && right.squaredNorm() > other.right.squaredNorm();
  const bool centerBetweenLeftRight = left.angle() < other.center.angle() && right.angle() > other.center.angle();
  return centerBetweenLeftRight && allPointsFurtherAway;
}

bool ObstacleHypothesis::isInImage(Vector2f& centerInImage, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix) const
{
  constexpr float inImageThreshold = 10.f;
  return Transformation::robotToImage(center, theCameraMatrix, theCameraInfo, centerInImage)
         && centerInImage.x() < theCameraInfo.width - inImageThreshold && centerInImage.x() > inImageThreshold
         && centerInImage.y() < theCameraInfo.height - inImageThreshold && centerInImage.y() > inImageThreshold;
}

void ObstacleHypothesis::dynamic(const float odometryRotation, const Vector2f& odometryTranslation, const Matrix2f& odometryJacobian,
                                 const float odometryNoiseX, const float odometryNoiseY)
{
  // Update the state.
  center.rotate(odometryRotation);
  left.rotate(odometryRotation);
  right.rotate(odometryRotation);
  velocity.rotate(odometryRotation);

  center += odometryTranslation;
  left += odometryTranslation;
  right += odometryTranslation;

  for(auto& observation : lastObservations)
  {
    observation.position.rotate(odometryRotation);
    observation.position += odometryTranslation;
  }

  // Process new covariance matrix.
  ASSERT(covariance(0, 1) == covariance(1, 0));
  covariance = odometryJacobian * covariance * odometryJacobian.transpose();
  covariance(0, 0) += odometryNoiseX;
  covariance(1, 1) += odometryNoiseY;
  Covariance::fixCovariance(covariance);
}

void ObstacleHypothesis::measurement(const ObstacleHypothesis& measurement, const float weightedSum,
                                     const float goalPostRadius)
{
  ASSERT(covariance(0, 1) == covariance(1, 0));
  Matrix2f newCov = measurement.covariance;
  newCov.noalias() += covariance;
  const Matrix2f& K = covariance * newCov.inverse();
  center += K* (measurement.center - center);
  covariance -= K* covariance;

  // SetLeftRight
  if(type == Obstacle::goalpost)
    setLeftRight(goalPostRadius);
  else
  {
    const float measurementWidth = (measurement.left - measurement.right).norm();
    const float obstacleWidth = (left - right).norm();
    float width = (obstacleWidth * (weightedSum - 1) + measurementWidth) / weightedSum;
    if(width < 2.f * Obstacle::getRobotDepth())
      width = 2.f * Obstacle::getRobotDepth();
    setLeftRight(width * .5f); // Radius (that's why * .5f)
  }
  Covariance::fixCovariance(covariance);
}

void ObstacleHypothesis::considerType(const ObstacleHypothesis& measurement, const int teamThreshold, const int uprightThreshold)
{
  team = std::min(sqr(teamThreshold), std::max(team + measurement.team, -sqr(teamThreshold)));
  upright = std::min(2 * uprightThreshold, std::max(upright + measurement.upright, -2 * uprightThreshold)); //'2' seems to be chosen wisely

  if(type == measurement.type || measurement.type == Obstacle::unknown || type == Obstacle::goalpost)
    return;

  if(type == Obstacle::unknown)
    type = measurement.type;
  else if(measurement.type == Obstacle::goalpost)
    type = Obstacle::goalpost;
  // the following code should perfectly consider whether a robot is fallen/upright and if it's an opponent or teammate
  else if(team <= -teamThreshold)
    type = upright <= -uprightThreshold ? Obstacle::fallenTeammate : Obstacle::teammate;
  else if(team >= teamThreshold)
    type = upright <= -uprightThreshold ? Obstacle::fallenOpponent : Obstacle::opponent;
  else
    type = upright <= -uprightThreshold ? Obstacle::fallenSomeRobot : Obstacle::someRobot;
}

bool ObstacleHypothesis::isFieldBoundaryFurtherAsObstacle(const CameraInfo& theCameraInfo,
                                                          const CameraMatrix& theCameraMatrix,
                                                          const ImageCoordinateSystem& theImageCoordinateSystem,
                                                          const FieldBoundary& theFieldBoundary)
{
  Vector2f obstacleLeftInImage, obstacleRightInImage;
  const bool hasLeft = Transformation::robotToImage(this->left, theCameraMatrix, theCameraInfo, obstacleLeftInImage);
  const bool hasRight = Transformation::robotToImage(this->right, theCameraMatrix, theCameraInfo, obstacleRightInImage);
  obstacleLeftInImage = theImageCoordinateSystem.fromCorrected(obstacleLeftInImage);
  obstacleRightInImage = theImageCoordinateSystem.fromCorrected(obstacleRightInImage);
  const int boundaryYLeft = theFieldBoundary.getBoundaryY(static_cast<int>(obstacleLeftInImage.x()));
  const int boundaryYRight = theFieldBoundary.getBoundaryY(static_cast<int>(obstacleRightInImage.x()));

  return hasLeft && hasRight
         && boundaryYLeft > obstacleLeftInImage.y()
         && boundaryYRight > obstacleRightInImage.y();
}

float ObstacleHypothesis::squaredMahalanobis(const ObstacleHypothesis& other) const
{
  const Eigen::Matrix<float, 2, 1> meanDiff = center - other.center;
  const Eigen::Matrix<float, 2, 2> combinedCovs = (covariance + other.covariance) * .5f;
  return meanDiff.transpose() * combinedCovs.inverse() * meanDiff;
}

float ObstacleHypothesis::calculateStdDevOfStaticObstacleHypothesis() const
{
  Vector2f mean = Vector2f::Zero();
  for(const auto& observation : lastObservations)
    mean += observation.position;
  mean /= static_cast<float>(lastObservations.size());

  float variance = 0.f;
  for(const auto& observation : lastObservations)
    variance += (observation.position - mean).squaredNorm();
  variance /= lastObservations.size();

  return std::sqrt(variance);
}

float ObstacleHypothesis::calculateStdDevOfMovingObstacleHypothesis(Vector2f& velocity, Matrix2f& velocityCovariance) const
{
  const std::size_t size = lastObservations.size();
  // The time difference of two consecutive lastObservations.
  Eigen::MatrixXf A((size - 1) * 2, 2);
  // Position differences between two consecutive lastObservations.
  Eigen::VectorXf z((size - 1) * 2);
  // Each of the diagonal 2x2 matrices contains the mean value of the covariances of two consecutive lastObservations.
  Eigen::MatrixXf Sigma = MatrixXf::Zero((size - 1) * 2, (size - 1) * 2);

  for(unsigned i = 0; i < static_cast<unsigned>(size - 1); i++)
  {
    const Vector2f dPos = lastObservations[i].position - lastObservations[i + 1].position;  // In mm
    const float dt = static_cast<float>(lastObservations[i].timestamp - lastObservations[i + 1].timestamp) / 1000.f; // In s
    const Matrix2f dCov = (lastObservations[i].covariance + lastObservations[i + 1].covariance) / 2;
    z(i * 2) = dPos.x();
    z(i * 2 + 1) = dPos.y();
    A(i * 2, 0) = dt;
    A(i * 2, 1) = 0.f;
    A(i * 2 + 1, 0) = 0.f;
    A(i * 2 + 1, 1) = dt;
    Sigma(2 * i, 2 * i) = dCov(0, 0);
    Sigma(2 * i, 2 * i + 1) = dCov(0, 1);
    Sigma(2 * i + 1, 2 * i) = dCov(1, 0);
    Sigma(2 * i + 1, 2 * i + 1) = dCov(1, 1);
  }

  if(Sigma.determinant() == 0)
    return std::numeric_limits<float>::max();

  velocityCovariance = ((A.transpose() * Sigma.inverse()) * A).inverse();
  Covariance::fixCovariance(velocityCovariance);
  velocity = velocityCovariance * A.transpose() * Sigma.inverse() * z; // mm/s

  ASSERT(velocity.allFinite());
  ASSERT(velocityCovariance.allFinite());

  // Compute residuals
  const Eigen::VectorXf residuals = A * velocity - z;
  const float meanError = residuals.norm() / std::sqrt(static_cast<float>(size));

  return meanError; // In mm
}
