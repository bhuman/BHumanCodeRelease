/**.
 * @file GlobalOpponentsHypothesis.cpp
 *
 * Implementation of a class that represents a possible obstacle.
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 * @author Michelle Gusev
 */

#include "GlobalOpponentsHypothesis.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Math/BHMath.h"
#include "Math/Covariance.h"
#include "Tools/Math/Transformation.h"

#include <algorithm>

GlobalOpponentsHypothesis::GlobalOpponentsHypothesis(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right,
                                       const unsigned lastSeen, const Type type, const unsigned seenCount)
  : seenCount(seenCount)
{
  ASSERT(covariance(1, 0) == covariance(0, 1));
  this->covariance = covariance;
  this->center = center;
  this->left = left;
  this->right = right;
  this->lastSeen = lastSeen;
  this->type = type;

  team = isTeammate() ? -1 : (isOpponent() ? 1 : 0);
  upright = type < Obstacle::fallenSomeRobot ? 1 : -1;
}

GlobalOpponentsHypothesis::GlobalOpponentsHypothesis(const Type type)
{
  this->type = type;

  team = isTeammate() ? -1 : (isOpponent() ? 1 : 0);
  upright = type < Obstacle::fallenSomeRobot ? 1 : -1;
}

bool GlobalOpponentsHypothesis::isBehind(const GlobalOpponentsHypothesis& other) const
{
  const bool allPointsFurtherAway = left.squaredNorm() > other.left.squaredNorm() && right.squaredNorm() > other.right.squaredNorm();
  const bool centerBetweenLeftRight = left.angle() < other.center.angle() && right.angle() > other.center.angle();
  return centerBetweenLeftRight && allPointsFurtherAway;
}

bool GlobalOpponentsHypothesis::isInImage(Vector2f& centerInImage, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix) const
{
  constexpr float inImageThreshold = 10.f;
  return Transformation::robotToImage(center, theCameraMatrix, theCameraInfo, centerInImage)
         && centerInImage.x() < theCameraInfo.width - inImageThreshold && centerInImage.x() > inImageThreshold
         && centerInImage.y() < theCameraInfo.height - inImageThreshold && centerInImage.y() > inImageThreshold;
}

void GlobalOpponentsHypothesis::dynamic(const float odometryRotation, const Vector2f& odometryTranslation, const Matrix2f& odometryJacobian,
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

  // Process new covariance matrix.
  ASSERT(covariance(0, 1) == covariance(1, 0));
  covariance = odometryJacobian * covariance * odometryJacobian.transpose();
  covariance(0, 0) += odometryNoiseX;
  covariance(1, 1) += odometryNoiseY;
  Covariance::fixCovariance<2>(covariance);
}

void GlobalOpponentsHypothesis::measurement(const GlobalOpponentsHypothesis& measurement, const float modelWidthWeighting)
{
  ASSERT(covariance(0, 1) == covariance(1, 0));
  Matrix2f newCov = measurement.covariance;
  newCov.noalias() += covariance;
  const Matrix2f& K = covariance * newCov.inverse();
  center += K* (measurement.center - center);
  covariance -= K* covariance;
  Covariance::fixCovariance<2>(covariance);

  // SetLeftRight
  const float measurementWidth = (measurement.left - measurement.right).norm();
  const float obstacleWidth = (left - right).norm();
  float width = (obstacleWidth * modelWidthWeighting + measurementWidth) / (modelWidthWeighting + 1);
  if(width < 2.f * Obstacle::getRobotDepth())
    width = 2.f * Obstacle::getRobotDepth();
  setLeftRight(width * .5f); // Radius (that's why * .5f)
}

void GlobalOpponentsHypothesis::determineAndSetType(const GlobalOpponentsHypothesis& measurement, const int teamThreshold, const int uprightThreshold)
{
  team = std::min(sqr(teamThreshold), std::max(team + measurement.team, -sqr(teamThreshold)));
  upright = std::min(2 * uprightThreshold, std::max(upright + measurement.upright, -2 * uprightThreshold)); //'2' seems to be chosen wisely

  if(type == measurement.type || measurement.type == Obstacle::unknown)
    return;

  if(type == Obstacle::unknown)
    type = measurement.type;
  // the following code should perfectly consider whether a robot is fallen/upright and if it's an opponent or teammate
  else if(team <= -teamThreshold)
    type = upright <= -uprightThreshold ? Obstacle::fallenTeammate : Obstacle::teammate;
  else if(team >= teamThreshold)
    type = upright <= -uprightThreshold ? Obstacle::fallenOpponent : Obstacle::opponent;
  else
    type = upright <= -uprightThreshold ? Obstacle::fallenSomeRobot : Obstacle::someRobot;
}

bool GlobalOpponentsHypothesis::isFieldBoundaryFurtherAsObstacle(const CameraInfo& theCameraInfo,
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

float GlobalOpponentsHypothesis::squaredMahalanobis(const GlobalOpponentsHypothesis& other) const
{
  const Eigen::Matrix<float, 2, 1> meanDiff = center - other.center;
  const Eigen::Matrix<float, 2, 2> combinedCovs = (covariance + other.covariance) * .5f;
  return meanDiff.transpose() * combinedCovs.inverse() * meanDiff;
}
