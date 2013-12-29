/**
* @file ObstacleCombinator.cpp
*
* This file implements a module that merges information from the ultrasonic obstacle grid
* and perceptions from vision.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
* @author Nico Lehmann
*/

#include "ObstacleCombinator.h"

void ObstacleCombinator::update(ObstacleModel& obstacleModel)
{
  obstacleModel.obstacles = theUSObstacleModel.obstacles;
  addFootObstacles(obstacleModel);
  addArmObstacles(obstacleModel);

  // Robot dimensions:
  const float robotHeight = 580;
  const float robotWidth  = 300; // Guessed value
//  const float robotDepth  = 150; // Guessed value

  // Add robot obstacles:
  const float maxRobotDistanceSqr = (float) sqr(maxRobotDistance);
  for(RobotsModel::RCIt robot = theRobotsModel.robots.begin(); robot != theRobotsModel.robots.end(); robot++)
  {
    const float robotDistanceSqr = robot->relPosOnField.squareAbs();
    if(((!robot->standing && considerLyingRobots)
        && robotDistanceSqr < maxRobotDistanceSqr && maxRobotDistanceSqr > 0.f) ||
        ((robot->standing && considerStandingRobots)
         && robotDistanceSqr < maxRobotDistanceSqr && maxRobotDistanceSqr > 0.f))
    {
      const float robotDistance = std::sqrt(robotDistanceSqr);
      const float robotDistanceInv = 1.f / robotDistance;
      const Vector2<> widthOffset((robot->standing ? robotWidth : robotHeight) * 0.5f, 0.0f);
      const Vector2<>& robotCenter = robot->relPosOnField;
      const Vector2<> robotCenterNorm = robotCenter * robotDistanceInv;
      const Vector2<> closestRobotPoint = robotCenter * (1.f - widthOffset.x * robotDistanceInv);
      Vector2<> vertDirLeft = robotCenterNorm * widthOffset.x;
      vertDirLeft.rotateLeft();
      const Vector2<> leftCorner = closestRobotPoint + vertDirLeft;
      const Vector2<> rightCorner = closestRobotPoint - vertDirLeft;
      obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, robotCenter, closestRobotPoint,
                                        robot->covariance, ObstacleModel::Obstacle::ROBOT));
    }
  }

  EXECUTE_ONLY_IN_DEBUG(obstacleModel.draw3D(theTorsoMatrix););
}


// TODO add some parameters
void ObstacleCombinator::addFootObstacles(ObstacleModel& obstacleModel)
{
  if(theFrameInfo.getTimeSince(theFootContactModel.lastContactLeft) < 2000)
  {
    Vector2<float> leftCorner(100.f,200.f);
    Vector2<float> rightCorner(100.f,0.f);
    Vector2<float> center(100.f,100.f);
    Vector2<float> closestPoint(rightCorner);
    Matrix2x2<> covariance(400, 0, 0, 400);
    obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, center, closestPoint,
      covariance, ObstacleModel::Obstacle::FOOT));
  }
  if(theFrameInfo.getTimeSince(theFootContactModel.lastContactRight) < 2000)
  {
    Vector2<float> rightCorner(100.f,-200.f);
    Vector2<float> leftCorner(100.f,0.f);
    Vector2<float> center(100.f,-100.f);
    Vector2<float> closestPoint(leftCorner);
    Matrix2x2<> covariance(400, 0, 0, 400);
    obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, center, closestPoint,
      covariance, ObstacleModel::Obstacle::FOOT));
  }
}


void ObstacleCombinator::addArmObstacles(ObstacleModel& obstacleModel)
{
  if(theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactLeft) < 2000)
  {
    addArmObstacle(obstacleModel, true, theArmContactModel.pushDirectionLeft);
  }
  if(theFrameInfo.getTimeSince(theArmContactModel.timeOfLastContactRight) < 2000)
  {
    addArmObstacle(obstacleModel, false, theArmContactModel.pushDirectionRight);
  }
}


// This implementation is quite simplistic and deliberately ignores odometry => but the approach works
void ObstacleCombinator::addArmObstacle(ObstacleModel& obstacleModel, bool leftArm,
  const ArmContactModel::PushDirection& pushDirection)
{
  // ignore pushed from "inside":
  if((leftArm && pushDirection == ArmContactModel::W) || (!leftArm && pushDirection == ArmContactModel::E))
    return;
  // enter obstacle before or behind me:
  float x(100.f);
  if(pushDirection == ArmContactModel::NE || pushDirection == ArmContactModel::N || pushDirection == ArmContactModel::NW)
    x = -100.f;
  Vector2<float> rightCorner(x, leftArm ? 0.f : -200.f);
  Vector2<float> leftCorner(x, leftArm ? 200.f : 0.f);
  Vector2<float> center(x, leftArm ? 100.f : -100.f);
  Vector2<float> closestPoint(leftArm ? rightCorner : leftCorner);
  Matrix2x2<> covariance(400, 0, 0, 400);
  obstacleModel.obstacles.push_back(ObstacleModel::Obstacle(leftCorner, rightCorner, center, closestPoint,
    covariance, ObstacleModel::Obstacle::ARM));
}


MAKE_MODULE(ObstacleCombinator, Modeling)
