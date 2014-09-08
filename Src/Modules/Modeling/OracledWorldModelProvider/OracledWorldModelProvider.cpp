/**
* @file Modules/Infrastructure/OracledWorldModelProvider.h
*
* This file implements a module that provides models based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#include "OracledWorldModelProvider.h"

OracledWorldModelProvider::OracledWorldModelProvider():
lastBallModelComputation(0), lastRobotPoseComputation(0)
{
}

void OracledWorldModelProvider::computeRobotPose()
{
  if(lastRobotPoseComputation == theFrameInfo.time)
    return;
  DRAW_ROBOT_POSE("module:OracledWorldModelProvider:realRobotPose", theGroundTruthWorldState.ownPose, ColorRGBA::magenta);
  theRobotPose = theGroundTruthWorldState.ownPose + robotPoseOffset;
  theRobotPose.deviation = 1.f;
  theRobotPose.validity = 1.f;
  lastRobotPoseComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::computeBallModel()
{
  if(lastBallModelComputation == theFrameInfo.time || theGroundTruthWorldState.balls.size() == 0)
    return;
  computeRobotPose();
  Vector2<> ballPosition = theGroundTruthWorldState.balls[0];

  Vector2<float> velocity((ballPosition - lastBallPosition) / float(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) * 1000.0f);
  theBallModel.estimate.position = theRobotPose.invert() * ballPosition;
  theBallModel.estimate.velocity = Vector2<>(velocity).rotate(-theRobotPose.rotation);
  theBallModel.lastPerception = theBallModel.estimate.position;
  theBallModel.timeWhenLastSeen = theFrameInfo.time;
  theBallModel.timeWhenDisappeared = theFrameInfo.time;

  lastBallPosition = ballPosition;
  lastBallModelComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::update(BallModel& ballModel)
{
  computeBallModel();
  ballModel = theBallModel;
}

void OracledWorldModelProvider::update(GroundTruthBallModel& groundTruthBallModel)
{
  computeBallModel();
  groundTruthBallModel.lastPerception = theBallModel.lastPerception;
  groundTruthBallModel.estimate = theBallModel.estimate;
  groundTruthBallModel.timeWhenDisappeared = theBallModel.timeWhenDisappeared;
  groundTruthBallModel.timeWhenLastSeen = theBallModel.timeWhenLastSeen;
}

void OracledWorldModelProvider::update(ObstacleModel& obstacleModel)
{
  computeRobotPose();
  obstacleModel.obstacles.clear();
  for(unsigned int i = 0; i<theGroundTruthWorldState.blueRobots.size(); ++i)
    robotToObstacle(theGroundTruthWorldState.blueRobots[i], obstacleModel);
  for(unsigned int i = 0; i<theGroundTruthWorldState.redRobots.size(); ++i)
    robotToObstacle(theGroundTruthWorldState.redRobots[i],obstacleModel);
}

void OracledWorldModelProvider::update(ExpObstacleModel& expObstacleModel)
{
  computeRobotPose();
  expObstacleModel.eobs.clear();
  for(unsigned int i = 0; i < theGroundTruthWorldState.blueRobots.size(); ++i)
    robotToObstacle(theGroundTruthWorldState.blueRobots[i], expObstacleModel, false);
  for(unsigned int i = 0; i < theGroundTruthWorldState.redRobots.size(); ++i)
    robotToObstacle(theGroundTruthWorldState.redRobots[i], expObstacleModel, true);
}

void OracledWorldModelProvider::robotToObstacle(const GroundTruthWorldState::GroundTruthRobot& robot,ObstacleModel& obstacleModel) const
{
  const float robotRadius(120.f);   // Hardcoded as this code will vanish as soon as the new obstacle model has been finished
  ObstacleModel::Obstacle obstacle;
  obstacle.center = theRobotPose.invert() * robot.pose.translation;
  obstacle.leftCorner = obstacle.rightCorner = obstacle.closestPoint = obstacle.center;
  float distance = obstacle.center.abs();
  if(distance > robotRadius)
  {
    obstacle.closestPoint.normalize();
    obstacle.closestPoint *= (distance - robotRadius);
  }
  Vector2<> leftOffset = obstacle.center;
  leftOffset.normalize();
  leftOffset *= robotRadius;
  Vector2<> rightOffset = leftOffset;
  leftOffset.rotateLeft();
  rightOffset.rotateRight();
  obstacle.leftCorner += leftOffset;
  obstacle.rightCorner += rightOffset;
  obstacle.type = ObstacleModel::Obstacle::ROBOT;
  obstacleModel.obstacles.push_back(obstacle);
}

void OracledWorldModelProvider::robotToObstacle(const GroundTruthWorldState::GroundTruthRobot& robot, ExpObstacleModel& expObstacleModel, const bool isRed) const
{
  ExpObstacleModel::ExpObstacle obstacle;
  obstacle.center = theRobotPose.invert() * robot.pose.translation;
  obstacle.type = isRed ? ExpObstacleModel::ExpObstacle::ROBOTRED : ExpObstacleModel::ExpObstacle::ROBOTBLUE;
  obstacle.lastMeasurement = theFrameInfo.time;
  obstacle.seenCount = 14;
  obstacle.velocity = Vector2<>(1.f, 1.f);
  expObstacleModel.eobs.emplace_back(obstacle);
}

void OracledWorldModelProvider::update(RobotPose& robotPose)
{
  DECLARE_DEBUG_DRAWING("module:OracledWorldModelProvider:realRobotPose", "drawingOnField");
  computeRobotPose();
  robotPose = theRobotPose;
}

void OracledWorldModelProvider::update(GroundTruthRobotPose& groundTruthRobotPose)
{
  computeRobotPose();
  groundTruthRobotPose.translation = theRobotPose.translation;
  groundTruthRobotPose.rotation = theRobotPose.rotation;
  groundTruthRobotPose.covariance = theRobotPose.covariance;
  groundTruthRobotPose.deviation = theRobotPose.deviation;
  groundTruthRobotPose.validity = theRobotPose.validity;
  groundTruthRobotPose.timestamp = theFrameInfo.time;
}

MAKE_MODULE(OracledWorldModelProvider,Cognition Infrastructure)
