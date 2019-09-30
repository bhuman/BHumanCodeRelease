/**
 * @file TeamPlayersLocator.cpp
 *
 * Implements a class that provides information about player positions on the field.
 * The model is a fusion of own observations as well as of observations by teammates.
 *
 * @author Katharina Gillmann
 * @pfuscher Florian
 */

#include "TeamPlayersLocator.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Modeling/Obstacle.h"
#include <algorithm>
#include <limits>

#define GOALPOST Obstacle::goalpost

MAKE_MODULE(TeamPlayersLocator, modeling)

TeamPlayersLocator::TeamPlayersLocator()
{
  goalPosts.reserve(4);
  goalPosts.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal), 0, GOALPOST);
  goalPosts.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal), 0, GOALPOST);
  goalPosts.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal), 0, GOALPOST);
  goalPosts.emplace_back(Matrix2f::Identity(), Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal), 0, GOALPOST);
}

void TeamPlayersLocator::update(TeamPlayersModel& teamPlayersModel)
{
  auto& obstacles = teamPlayersModel.obstacles;
  DECLARE_DEBUG_DRAWING("module:TeamPlayersLocator:others", "drawingOnField");
  obstacles.clear();
  ownTeam.clear();

  if(theRobotInfo.penalty != PENALTY_NONE || !theGroundContactState.contact || theFallDownState.state != theFallDownState.upright)
    return;

  std::vector<std::pair<Obstacle, unsigned char>, Eigen::aligned_allocator<std::pair<Obstacle, unsigned char>>> others;

  for(auto& goalPost : goalPosts)
  {
    obstacles.push_back(goalPost);
  }

  ownTeam[theRobotInfo.number] = theRobotPose;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status == Teammate::PLAYING)
    {
      if(teammate.theRobotPose.deviation < teammatePoseDeviation)
      {
        ownTeam[teammate.number] = teammate.theRobotPose; //position of teammates
      }
    }
  }

  for(const auto& obstacle : theObstacleModel.obstacles)
  {
    if(obstacle.type == GOALPOST)
      continue;
    //if seen robots are detected inside the field
    const Vector2f p = theRobotPose * obstacle.center;
    if(std::abs(p.x()) < theFieldDimensions.xPosOpponentFieldBorder && std::abs(p.y()) < theFieldDimensions.yPosLeftFieldBorder)
    {
      if(isGoalPost(p) || isTeammate(p, squaredDistanceThreshold, theRobotInfo.number))
        continue;
      Matrix2f covariance = Covariance::rotateCovarianceMatrix(obstacle.covariance, theRobotPose.rotation);
      Covariance::fixCovariance(covariance);
      if(isInsideOwnDetectionArea(p, theRobotInfo.number, obstacle.lastSeen))
        obstacles.emplace_back(covariance, p, obstacle.lastSeen, obstacle.type);
      else
        others.emplace_back(Obstacle(covariance, p, obstacle.lastSeen, obstacle.type), 0);
    }
  }

  const size_t endOwnObstacleMarker = obstacles.size();

  STOPWATCH("TeamPlayersLocator:obstaclesByTeammates")
  {
    for(auto const& teammate : theTeamData.teammates)
    {
      if(teammate.status == Teammate::PLAYING && teammate.theRobotPose.deviation < teammatePoseDeviation)
      {
        for(const auto& obstacle : teammate.theObstacleModel.obstacles)
        {
          if(obstacle.type == GOALPOST)
            continue;
          //if seen robots are detected inside the field
          const Vector2f p = teammate.theRobotPose * obstacle.center.cast<float>();
          if(std::abs(p.x()) < theFieldDimensions.xPosOpponentFieldBorder && std::abs(p.y()) < theFieldDimensions.yPosLeftFieldBorder)
          {
            if(isGoalPost(p) || isTeammate(p, squaredDistanceThreshold, teammate.number))
              continue;
            Matrix2f covariance = Covariance::rotateCovarianceMatrix(obstacle.covariance, teammate.theRobotPose.rotation);
            Covariance::fixCovariance(covariance);
            if(isInsideOwnDetectionArea(p, teammate.number, obstacle.lastSeen)
               && !collideOtherDetectionArea(p, teammate.number, ownTeam, obstacle.center.cast<float>().squaredNorm()))
              obstacles.emplace_back(covariance, p, obstacle.lastSeen, obstacle.type);
            else
              others.emplace_back(Obstacle(covariance, p, obstacle.lastSeen, obstacle.type), 0);
          }
        }
      }
    }
  }

  size_t obstacleSizeBeforeOthers = obstacles.size();
  //do the cluster thing
  STOPWATCH("TeamPlayersLocator:clusterThing")
  {
    //cluster the others
    for(size_t obstacle = 0; obstacle < others.size(); ++obstacle)
    {
      for(size_t otherObstacle = others.size() - 1; otherObstacle > obstacle; --otherObstacle)
      {
        if((others[obstacle].first.center - others[otherObstacle].first.center).squaredNorm() < squaredDistanceThreshold)
        {
          Obstacle::fusion2D(others[obstacle].first, others[otherObstacle].first);
          ++others[obstacle].second;
          setType(others[obstacle].first, others[otherObstacle].first);
          others[obstacle].first.lastSeen = std::max(others[obstacle].first.lastSeen, others[otherObstacle].first.lastSeen);
          others.erase(others.begin() + otherObstacle);
        }
      }
    }
    for(const auto& other : others)
    {
      if(other.second > 0 && !isTeammate(other.first.center, squaredDistanceThreshold))
        obstacles.push_back(other.first);
      else
        CIRCLE("module:TeamPlayersLocator:others", other.first.center.x(), other.first.center.y(), 100, 40, Drawings::dottedPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::black);
    }
    //                                 by Jesse: we should have clustered the others before.
    for(size_t obstacle = 0; obstacle < obstacleSizeBeforeOthers; ++obstacle)
    {
      //                                                                        by Jesse: never merge our own obstacles together
      for(size_t otherObstacle = obstacles.size() - 1; otherObstacle > obstacle && otherObstacle >= endOwnObstacleMarker; --otherObstacle)
      {
        float obstaclesDistance = (obstacles[obstacle].center - obstacles[otherObstacle].center).squaredNorm();
        if(obstacles[obstacle].type == GOALPOST && obstacles[otherObstacle].type != GOALPOST && obstaclesDistance < squaredDistanceGoalPostThreshold)
        {
          obstacles[obstacle].lastSeen = std::max(obstacles[obstacle].lastSeen, obstacles[otherObstacle].lastSeen);
          obstacles.erase(obstacles.begin() + otherObstacle);
        }
        else if(obstaclesDistance < squaredDistanceThreshold && obstacles[obstacle].type != GOALPOST && obstacles[otherObstacle].type != GOALPOST)
        {
          // by Jesse: if both obstacles are seen in a primary sector assert a doubled accuracy
          if(otherObstacle < obstacleSizeBeforeOthers && obstaclesDistance > squaredDistanceThreshold / sqr(2))
            continue;

          // by Jesse: if an obstacle is observed by the robot itself i don't want to alter that position observation
          //           if an obstacle is "save" obseved by an other robot i don't want to alter it with an 'other'-obstacle, too.
          if(!(obstacle < endOwnObstacleMarker || (obstacle < obstacleSizeBeforeOthers && otherObstacle >= obstacleSizeBeforeOthers)))
            Obstacle::fusion2D(obstacles[obstacle], obstacles[otherObstacle]);

          setType(obstacles[obstacle], obstacles[otherObstacle]);
          obstacles[obstacle].lastSeen = std::max(obstacles[obstacle].lastSeen, obstacles[otherObstacle].lastSeen);
          obstacles.erase(obstacles.begin() + otherObstacle);
          if(otherObstacle < obstacleSizeBeforeOthers)
            --obstacleSizeBeforeOthers;
        }
      }
    }
  }

  for(const auto& tm : ownTeam)
  {
    if(tm.first == theRobotInfo.number)
      continue;
    obstacles.emplace_back(tm.second.covariance.topLeftCorner(2, 2), tm.second.translation, theFrameInfo.time);
  }
}

bool TeamPlayersLocator::isInsideOwnDetectionArea(const Vector2f& position, int robotNumber, int lastSeen) const
{
  float tmp;
  return isInsideOwnDetectionArea(position, robotNumber, tmp, lastSeen);
}

bool TeamPlayersLocator::isInsideOwnDetectionArea(const Vector2f& position, int robotNumber, float& distance, int lastSeen) const
{
  //obstacles behind the robot are okay
  Vector2f point = ownTeam.at(robotNumber).inversePose * position;
  distance = point.squaredNorm();
  return (theFrameInfo.getTimeSince(lastSeen) < obstacleAgeThreshold || point.x() > -2.f * Obstacle::getRobotDepth())
         && distance <= sqr(selfDetectionOnlyRadius);
}

bool TeamPlayersLocator::collideOtherDetectionArea(const Vector2f& position, int robotNumber, std::map<int, RobotPose>& ownTeam,
                                                   const float distance) const
{
  float tempDistance;
  for(const auto& otherPlayer : ownTeam)
  {
    if(otherPlayer.first == robotNumber)
      continue;
    if(isInsideOwnDetectionArea(position, otherPlayer.first, tempDistance, obstacleAgeThreshold) && tempDistance < distance)
      return true;
  }
  return false;
}

bool TeamPlayersLocator::isGoalPost(const Vector2f& position) const
{
  for(const auto& gP : goalPosts)
  {
    if((position - gP.center).squaredNorm() < sqr(2.f * theFieldDimensions.goalPostRadius))
      return true;
  }
  return false;
}

bool TeamPlayersLocator::isTeammate(const Vector2f& position, const float radius, int ignoreRobotNumber) const
{
  for(const auto& tm : ownTeam)
  {
    if(ignoreRobotNumber == tm.first)
      continue;
    if((position - tm.second.translation).squaredNorm() < radius)
      return true;
  }
  return false;
}

bool TeamPlayersLocator::isTeammate(const Vector2f& position, const float radius) const
{
  for(const auto& tm : ownTeam)
  {
    if((position - tm.second.translation).squaredNorm() < radius)
      return true;
  }
  return false;
}

void TeamPlayersLocator::setType(Obstacle& one, const Obstacle& other) const
{
  if(one.type != other.type && other.type != Obstacle::unknown)
  {
    if(one.type == Obstacle::unknown)
      one.type = other.type;
    else if(one.type >= Obstacle::fallenSomeRobot && other.type >= Obstacle::fallenSomeRobot)
    {
      if(one.type == Obstacle::fallenSomeRobot)
        one.type = other.type; // the other robot knows more about the color than I do
      else if(other.type != Obstacle::fallenSomeRobot)
        one.type = Obstacle::fallenSomeRobot; // both robots disagree
    }
    else if(one.type >= Obstacle::someRobot && other.type >= Obstacle::someRobot)
    {
      const Obstacle::Type oneTypeWithoutFallen = one.type >= Obstacle::fallenSomeRobot ? static_cast<Obstacle::Type>(one.type - 3) : one.type;
      const Obstacle::Type otherTypeWithoutFallen = other.type >= Obstacle::fallenSomeRobot ? static_cast<Obstacle::Type>(other.type - 3) : other.type;
      if(oneTypeWithoutFallen == Obstacle::someRobot)
        one.type = otherTypeWithoutFallen;
      else if(oneTypeWithoutFallen != Obstacle::someRobot && oneTypeWithoutFallen != otherTypeWithoutFallen)
        one.type = Obstacle::someRobot;
    }
  }
}
