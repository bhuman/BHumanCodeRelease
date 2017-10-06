/**.
 *
 * @author <a href="mailto:flomaass@informatik.uni-bremen.de">Florian Maaß</a>
 */

#include "InternalObstacle.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>
#include <set>

using namespace impl;

InternalObstacle::InternalObstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right,
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

InternalObstacle::InternalObstacle(const Type type)
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

void InternalObstacle::dynamic(const float odometryRotation, const Vector2f odometryTranslation, const Matrix2f odometryJacobian,
                               const float odometryNoiseX, const float odometryNoiseY, const float oKFDynamicNoise, const float dKFDynamicNoise)
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

  oKF.rotate(odometryRotation);
  orientation = oKF.getMean();
  oKF.predict(oKFDynamicNoise);
  dKF.predict(dKFDynamicNoise);
}

void InternalObstacle::measurement(const InternalObstacle& measurement, const float weightedSum,
                                   const FieldDimensions& theFieldDimensions, const float oKFMeasureNoise, const float dKFMeasureNoise)
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
  muX.noalias() += K * (measurement.center - center);
  covariance -= K * covariance;

  center << muX(0), muX(1);

  float measurementOrientation = measurement.orientation;
  // if there is an orientation value in the measurement:
  if(detectedOrientation == 2 || (detectedOrientation == 1 && measurement.detectedOrientation == 2))
  {
    float distance = std::min(std::abs(orientation - measurementOrientation),
                              std::abs((orientation > measurementOrientation ? orientation - (2 * pi) : measurementOrientation - (2 * pi)) -
                                       (orientation > measurementOrientation ? measurementOrientation : orientation)));

    // if in state, front-/ back-facing is known but in measurement not:
    if(measurement.detectedOrientation == 1)
    {
      if(distance > pi / 2)
      {
        measurementOrientation += measurementOrientation > 0 ? -pi : pi;
      }
    }

    bool measurementWrongFacing = false;
    // if measurement knows facing direction, update the direction Kalman Filter:
    if(measurement.detectedOrientation == 2)
    {
      float distance = std::min(std::abs(orientation - measurementOrientation),
                                std::abs((orientation > measurementOrientation ? orientation - (2 * pi) : measurementOrientation - (2 * pi)) -
                                         (orientation > measurementOrientation ? measurementOrientation : orientation)));

      if(distance > pi / 2)
      {
        measurementWrongFacing = true;
        dKF.update(1.f, dKFMeasureNoise);

        // if there were many wrong direction, turn direction:
        if(dKF.getMean() > 0.5)
        {
          measurementWrongFacing = false;
        }
      }
      else
      {
        dKF.update(0.f, dKFMeasureNoise);
      }
    }

    // if measurement orientation is not zero, do things:
    // Here has to be added the kalman filter:
    if(measurement.detectedOrientation != 0)
    {
      // if measurement shows very shure in the false direction, mirror it:
      if(measurementWrongFacing)
      {
        measurementOrientation += measurementOrientation > 0 ? -pi : pi;
      }

      oKF.update(measurementOrientation, oKFMeasureNoise);

      orientation = oKF.getMean();
      detectedOrientation = 2;
    }
  }
  else
  {
    // if there is no orientation until now, just set the values by measurement:
    orientation = measurement.orientation;
    detectedOrientation = measurement.detectedOrientation;
  }

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

void InternalObstacle::considerType(const InternalObstacle& measurement, const int colorThreshold, const int uprightThreshold)
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

bool InternalObstacle::isBehind(const InternalObstacle& other) const
{
  const bool allPointsFurtherAway = left.squaredNorm() > other.left.squaredNorm() && right.squaredNorm() > other.right.squaredNorm();
  const bool centerBetweenLeftRight = left.angle() < other.center.angle() && right.angle() > other.center.angle();
  return centerBetweenLeftRight && allPointsFurtherAway;
}

bool InternalObstacle::isInImage(Vector2f& centerInImage, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix) const
{
  return Transformation::robotToImage(center, theCameraMatrix, theCameraInfo, centerInImage)
         && centerInImage.x() < theCameraInfo.width - 10.f && centerInImage.x() > 10.f
         && centerInImage.y() < theCameraInfo.height - 10.f && centerInImage.y() > 10.f;
}

//use the boundary spots to make sure the absence of an obstacle (boundary spots are at the edge from green to garbage)
//the center of an obstacle is probably on a green scanline, so use the width of an obstacle and check if there are more
//points below the resulting line (obstacle left to right)
bool InternalObstacle::fieldBoundaryFurtherAsObstacle(const Vector2f& centerInImage, const unsigned notSeenThreshold,
  const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix, const FieldBoundary& theFieldBoundary)
{
  Vector2f leftInImage, rightInImage;
  const bool left = Transformation::robotToImage(this->left, theCameraMatrix, theCameraInfo, leftInImage);
  const bool right = Transformation::robotToImage(this->right, theCameraMatrix, theCameraInfo, rightInImage);
  int result = 0;
  for(const auto& boundarySpot : theFieldBoundary.boundarySpots)
  {
    //obstacle is behind the field boundary (obstacle could be seen but is not present)
    if(left && right && boundarySpot.x() >= leftInImage.x() && boundarySpot.x() <= rightInImage.x())
    {
      if(boundarySpot.y() < leftInImage.y() && boundarySpot.y() < rightInImage.y())
        --result;
      else
        ++result;
    }
  }
  if(result < -1)
  {
    notSeenButShouldSeenCount += std::max(1u, notSeenThreshold / 10);
    return true;
  }

  return false;
}
