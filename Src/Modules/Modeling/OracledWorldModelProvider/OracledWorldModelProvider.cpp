/**
* @file Modules/Infrastructure/OracledWorldModelProvider.h
*
* This file implements a module that provides models based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#include "OracledWorldModelProvider.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

OracledWorldModelProvider::OracledWorldModelProvider():
  lastBallModelComputation(0), lastRobotPoseComputation(0)
{}

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
  Vector2f ballPosition = theGroundTruthWorldState.balls[0];

  Vector2f velocity((ballPosition - lastBallPosition) / float(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) * 1000.f);
  theBallModel.estimate.position = theRobotPose.inverse() * ballPosition;
  theBallModel.estimate.velocity = velocity.rotate(-theRobotPose.rotation);
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
  if(!Global::settingsExist())
    return;

  // Simulation scene should only use blue and red for now
  ASSERT(Global::getSettings().teamColor == Settings::blue || Global::getSettings().teamColor == Settings::red);

  const bool teammate = Global::getSettings().teamColor == Settings::blue;
  for(unsigned int i = 0; i < theGroundTruthWorldState.bluePlayers.size(); ++i)
    playerToObstacle(theGroundTruthWorldState.bluePlayers[i], obstacleModel, teammate);
  for(unsigned int i = 0; i < theGroundTruthWorldState.redPlayers.size(); ++i)
    playerToObstacle(theGroundTruthWorldState.redPlayers[i], obstacleModel, !teammate);

  //add goal posts
  float squaredObstacleModelMaxDistance = sqr(obstacleModelMaxDistance);
  Vector2f goalPost = theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  if(goalPost.squaredNorm() < squaredObstacleModelMaxDistance)
    obstacleModel.obstacles.emplace_back(Matrix2f::Identity(), goalPost, Obstacle::goalpost);
  goalPost = theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  if(goalPost.squaredNorm() < squaredObstacleModelMaxDistance)
    obstacleModel.obstacles.emplace_back(Matrix2f::Identity(), goalPost, Obstacle::goalpost);
  goalPost = theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  if(goalPost.squaredNorm() < squaredObstacleModelMaxDistance)
    obstacleModel.obstacles.emplace_back(Matrix2f::Identity(), goalPost, Obstacle::goalpost);
  goalPost = theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  if(goalPost.squaredNorm() < squaredObstacleModelMaxDistance)
    obstacleModel.obstacles.emplace_back(Matrix2f::Identity(), goalPost, Obstacle::goalpost);
}

void OracledWorldModelProvider::playerToObstacle(const GroundTruthWorldState::GroundTruthPlayer& player, ObstacleModel& obstacleModel, const bool isTeammate) const
{
  Vector2f center(theRobotPose.inverse() * player.pose.translation);
  if(center.squaredNorm() >= sqr(obstacleModelMaxDistance))
    return;
  Obstacle obstacle(Matrix2f::Identity(), center,
                    isTeammate ? (player.upright ? Obstacle::teammate : Obstacle::fallenTeammate)
                               : player.upright ? Obstacle::opponent : Obstacle::fallenOpponent);
  obstacle.setLeftRight(Obstacle::getRobotDepth());
  obstacleModel.obstacles.emplace_back(obstacle);
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

MAKE_MODULE(OracledWorldModelProvider, cognitionInfrastructure)
