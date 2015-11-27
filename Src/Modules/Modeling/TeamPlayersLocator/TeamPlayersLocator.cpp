/**
* @file TeamPlayersLocator.cpp
*
* Implements a class that provides information about player positions on the field.
* The model is a fusion of own observations as well as of observations by teammates.
*
* @author Katharina Gillmann
* @author Florian Maaß
*/

#include "TeamPlayersLocator.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>

#define GOALPOST Obstacle::goalpost

MAKE_MODULE(TeamPlayersLocator, modeling)

void TeamPlayersLocator::update(TeamPlayersModel& teamPlayersModel)
{
  teamPlayersModel.obstacles.clear();

  std::vector<Obstacle> ownTeam;

  teamPlayersModel.obstacles.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal), GOALPOST);
  teamPlayersModel.obstacles.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal), GOALPOST);
  teamPlayersModel.obstacles.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal), GOALPOST);
  teamPlayersModel.obstacles.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal), GOALPOST);

  if(theRobotInfo.penalty == PENALTY_NONE && theGroundContactState.contact)
  {
    ownTeam.emplace_back(theRobotPose.covariance.topLeftCorner(2, 2), theRobotPose.translation); // pose and covariance of the robot itself

    if(theFallDownState.state == theFallDownState.upright) //why is this?
      for(const auto& obstacle : theObstacleModel.obstacles)
      {
        if(obstacle.type == GOALPOST)
          continue;
        // if seen robots are of the opponent team and are not detected outside the field
        const Vector2f p = theRobotPose * obstacle.center;
        if(std::abs(p.x()) <= theFieldDimensions.xPosOpponentFieldBorder && std::abs(p.y()) <= theFieldDimensions.yPosLeftFieldBorder)
          teamPlayersModel.obstacles.emplace_back(obstacle.covariance, p, obstacle.type);
      }
  }

  for(auto const& teammate : theTeammateData.teammates)
  {
    if(teammate.status == Teammate::FULLY_ACTIVE)
    {
      if(teammate.pose.deviation < 50.f) //todo, magic number will be replaced by parameter. 50.f might be too low
        ownTeam.emplace_back(teammate.pose.covariance.topLeftCorner(2, 2), teammate.pose.translation); //position of teammates

      for(const auto& obstacle : teammate.obstacleModel.obstacles)
      {
        if(obstacle.type == GOALPOST)
          continue;
        // if seen robots are of the opponent team and are not detected outside the field
        const Vector2f p = teammate.pose * obstacle.center;
        if(std::abs(p.x()) <= theFieldDimensions.xPosOpponentFieldBorder && std::abs(p.y()) <= theFieldDimensions.yPosLeftFieldBorder)
        {
          Matrix2f covariance = (Matrix2f() << obstacle.covXX, obstacle.covXY, obstacle.covXY, obstacle.covYY).finished();
          Obstacle converted = Obstacle(rotateCovariance(covariance, teammate.pose.rotation), p, obstacle.type);
          merge(converted, teamPlayersModel.obstacles);
        }
      }
    }
  }
  for(auto& teammate : ownTeam)
  {
    removeAround(teammate, teamPlayersModel.obstacles);
  }
}

Matrix2f TeamPlayersLocator::rotateCovariance(const Matrix2f& matrix, const float angle)
{
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  const Matrix2f rotationMatrix = (Matrix2f() << cosine, -sine, sine, cosine).finished();
  return (rotationMatrix * matrix) * rotationMatrix.transpose();
}

void TeamPlayersLocator::merge(Obstacle& obstacle, std::vector<Obstacle>& obstacles) const
{
  Obstacle* merge = nullptr;
  float bestDistance = std::numeric_limits<float>::max();
  for(auto& other : obstacles)
  {
    float squaredMahalanobisDistance = Covariance::squaredMahalanobisDistance(obstacle.center, obstacle.covariance + other.covariance, other.center);
    if((obstacle.center - other.center).squaredNorm() <= squaredDistanceThreshold || (squaredMahalanobisDistance < squaredMahalanobisDistanceParameter && squaredMahalanobisDistance < bestDistance))
    {
      bestDistance = squaredMahalanobisDistance;
      merge = &other;
    }
  }

  if(merge != nullptr && merge->type != Obstacle::goalpost)
  {
    Obstacle::fusion2D(*merge, obstacle);
    merge->type = setType(merge->type, obstacle.type);
  }
  else
  {
    if(!isInsideOwnDetectionArea(obstacle.center))
      obstacles.push_back(obstacle);
  }
}

void TeamPlayersLocator::removeAround(Obstacle& teammate, std::vector<Obstacle>& obstacles) const
{
  for(auto other = obstacles.begin(); other != obstacles.end();)
  {
    if(other->type == GOALPOST)
    {
      ++other;
      continue;
    }
    float squaredMahalanobisDistance = Covariance::squaredMahalanobisDistance(teammate.center, teammate.covariance + other->covariance, other->center);
    float squaredDistance = (other->center - teammate.center).squaredNorm();
    if(squaredDistance <= squaredDistanceThreshold || squaredMahalanobisDistance < squaredMahalanobisDistanceParameter)
    {
      other = obstacles.erase(other);
      continue;
    }
    ++other;
  }
  if(teammate.center != theRobotPose.translation)
    obstacles.push_back(teammate);
}

Obstacle::Type TeamPlayersLocator::setType(const Obstacle::Type one, const Obstacle::Type other) const
{
  ASSERT(one != Obstacle::goalpost);
  ASSERT(other != Obstacle::goalpost);
  if(one == other || other == Obstacle::unknown || one >= Obstacle::fallenSomeRobot)
    return one;
  if(one == Obstacle::unknown || other >= Obstacle::fallenSomeRobot)
    return other;

   return one;
}

bool TeamPlayersLocator::isInsideOwnDetectionArea(const Vector2f& position) const
{
  //obstacles behind the robot are okay
  Vector2f point = theRobotPose.inverse() * position;
  return point.x() > -Obstacle::getRobotDepth() && (point).squaredNorm() <= sqr(selfDetectionOnlyRadius);
}
