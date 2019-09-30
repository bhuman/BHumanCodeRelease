/**.
 *
 * @author <a href="mailto:flomaass@informatik.uni-bremen.de">Florian Maaß</a>
 */

#include "ObstacleHypothesis.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>

using namespace impl;

ObstacleHypothesis::ObstacleHypothesis(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right,
                                       const unsigned lastSeen, const unsigned seenCount, const Type type)
{
  this->covariance << covariance;
  ASSERT(covariance(1, 0) == covariance(0, 1));
  this->center = center;
  this->left = left;
  this->right = right;
  this->velocity.setZero();
  this->type = type;
  this->lastSeen = lastSeen;
  this->seenCount = seenCount;
  notSeenButShouldSeenCount = 0u;
  upright = type < Obstacle::fallenSomeRobot ? 1 : -1;
  if(isTeammate())
    color = -1;
  else if(isOpponent())
    color = 1;
  else
    color = 0;
}

ObstacleHypothesis::ObstacleHypothesis(const Type type)
{
  upright = type < Obstacle::fallenSomeRobot ? 1 : -1;
  if(isTeammate())
    color = -1;
  else if(isOpponent())
    color = 1;
  else
    color = 0;
  this->type = type;
}

void ObstacleHypothesis::dynamic(const float odometryRotation, const Vector2f odometryTranslation, const Matrix2f odometryJacobian,
                                 const float odometryNoiseX, const float odometryNoiseY)
{
  //update the state
  center.rotate(odometryRotation);
  left.rotate(odometryRotation);
  right.rotate(odometryRotation);

  velocity.setZero();
  center += odometryTranslation;
  left += odometryTranslation;
  right += odometryTranslation;

  //process new covariance matrix
  ASSERT(covariance(0, 1) == covariance(1, 0));
  covariance = odometryJacobian * covariance * odometryJacobian.transpose();
  covariance(0, 0) += odometryNoiseX;
  covariance(1, 1) += odometryNoiseY;
  Covariance::fixCovariance(covariance);
}

void ObstacleHypothesis::measurement(const ObstacleHypothesis& measurement, const float weightedSum,
                                     const FieldDimensions& theFieldDimensions)
{
  //covariance matrices
  //computation of kalman gain, new state and covariance matrix
  ASSERT(covariance(0, 1) == covariance(1, 0));
  Matrix2f CXZ = covariance;
  Matrix2f CZZ = measurement.covariance; //TODO: ...+CovZ
  CZZ.noalias() += CXZ;
  Matrix2f K = CXZ * CZZ.inverse();
  Vector2f muX;
  muX << center;
  muX.noalias() += K* (measurement.center - center);
  covariance -= K* covariance;

  center << muX(0), muX(1);

  if(type == Obstacle::goalpost)
  {
    setLeftRight(theFieldDimensions.goalPostRadius);
  }
  else
  {
    const float obstacleWidth = (left - right).norm();
    const float measurementWidth = (measurement.left - measurement.right).norm();
    float width = (measurementWidth + obstacleWidth * (weightedSum - 1)) / weightedSum;
    if(width < 2.f * Obstacle::getRobotDepth())
      width = 2.f * Obstacle::getRobotDepth();
    setLeftRight(width * .5f); //radius (that's why * .5f)
  }
  Covariance::fixCovariance(covariance);
}

void ObstacleHypothesis::considerType(const ObstacleHypothesis& measurement, const int colorThreshold, const int uprightThreshold)
{
  color = std::min(colorThreshold * colorThreshold, std::max(measurement.color + color, -colorThreshold * colorThreshold));
  upright = std::min(2 * uprightThreshold, std::max(measurement.upright + upright, -2 * uprightThreshold)); //'2' seems to be chosen wisely

  if(type == measurement.type || measurement.type == Obstacle::unknown)
    return;
  if(type == Obstacle::unknown)
  {
    type = measurement.type;
    return;
  }
  if(type == Obstacle::goalpost)
    return;
  if(measurement.type == Obstacle::goalpost)
  {
    type = Obstacle::goalpost;
    return;
  }

  //the following code should perfectly consider whether a robot is fallen/upright and if it's an oppponent or teammate
  if(color < colorThreshold && color > -colorThreshold)
  {
    if(upright <= -uprightThreshold)
    {
      type = Obstacle::fallenSomeRobot;
      return;
    }
    type = Obstacle::someRobot;
    return;
  }

  if(color >= colorThreshold)
  {
    if(upright <= -uprightThreshold)
    {
      type = Obstacle::fallenOpponent;
      return;
    }
    type = Obstacle::opponent;
    return;
  }
  else
  {
    if(upright <= -uprightThreshold)
    {
      type = Obstacle::fallenTeammate;
      return;
    }
    type = Obstacle::teammate;
    return;
  }
}

bool ObstacleHypothesis::isBehind(const ObstacleHypothesis& other) const
{
  const bool allPointsFurtherAway = left.squaredNorm() > other.left.squaredNorm() && right.squaredNorm() > other.right.squaredNorm();
  const bool centerBetweenLeftRight = left.angle() < other.center.angle() && right.angle() > other.center.angle();
  return centerBetweenLeftRight && allPointsFurtherAway;
}

bool ObstacleHypothesis::isInImage(Vector2f& centerInImage, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix) const
{
  return Transformation::robotToImage(center, theCameraMatrix, theCameraInfo, centerInImage)
         && centerInImage.x() < theCameraInfo.width - 10.f && centerInImage.x() > 10.f
         && centerInImage.y() < theCameraInfo.height - 10.f && centerInImage.y() > 10.f;
}

//use the boundary spots to make sure the absence of an obstacle (boundary spots are at the edge from green to garbage)
//the center of an obstacle is probably on a green scan line, so use the width of an obstacle and check if there are more
//points below the resulting line (obstacle left to right)
bool ObstacleHypothesis::fieldBoundaryFurtherAsObstacle(const Vector2f& centerInImage, const unsigned notSeenThreshold,
                                                        const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix,
                                                        const ImageCoordinateSystem& theImageCoordinateSystem,
                                                        const FieldBoundary& theFieldBoundary)
{
  Vector2f leftInImage, rightInImage;
  const bool left = Transformation::robotToImage(this->left, theCameraMatrix, theCameraInfo, leftInImage);
  const bool right = Transformation::robotToImage(this->right, theCameraMatrix, theCameraInfo, rightInImage);
  leftInImage = theImageCoordinateSystem.fromCorrected(leftInImage);
  rightInImage = theImageCoordinateSystem.fromCorrected(rightInImage);
  if(left && right
     && theFieldBoundary.getBoundaryY(static_cast<int>(leftInImage.x())) > leftInImage.y()
     && theFieldBoundary.getBoundaryY(static_cast<int>(rightInImage.x())) > rightInImage.y())
  {
    notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
    return true;
  }

  return false;
}
