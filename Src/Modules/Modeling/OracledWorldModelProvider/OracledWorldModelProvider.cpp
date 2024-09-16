/**
 * @file Modules/Infrastructure/OracledWorldModelProvider.cpp
 *
 * This file implements a module that provides models based on simulated data.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "OracledWorldModelProvider.h"
#include "Streaming/Global.h"
#include "Framework/Settings.h"
#include "Math/Pose3f.h"
#include "Debugging/DebugDrawings.h"

OracledWorldModelProvider::OracledWorldModelProvider():
  lastBallModelComputation(0), lastRobotPoseComputation(0)
{}

void OracledWorldModelProvider::computeRobotPose()
{
  if(lastRobotPoseComputation == theFrameInfo.time)
    return;
  DRAW_ROBOT_POSE("module:OracledWorldModelProvider:realRobotPose", theGroundTruthWorldState.ownPose, ColorRGBA::magenta);
  theRobotPose = theGroundTruthWorldState.ownPose + robotPoseOffset;
  theRobotPose.quality = RobotPose::superb;
  lastRobotPoseComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::computeBallModel()
{
  if(lastBallModelComputation == theFrameInfo.time || theGroundTruthWorldState.balls.size() == 0)
    return;
  computeRobotPose();
  const Vector2f ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
  const Vector2f ballVelocity = theGroundTruthWorldState.balls[0].velocity.head<2>();

  theBallModel.estimate.position = theRobotPose.inverse() * ballPosition;
  theBallModel.estimate.velocity = ballVelocity.rotated(-theRobotPose.rotation);
  theBallModel.lastPerception = theBallModel.estimate.position;
  theBallModel.timeWhenLastSeen = theFrameInfo.time;
  theBallModel.timeWhenDisappeared = theFrameInfo.time;

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

void OracledWorldModelProvider::update(TeammatesBallModel& teammatesBallModel)
{
  computeBallModel();
  computeRobotPose();
  teammatesBallModel.position = theRobotPose * theBallModel.estimate.position;
  teammatesBallModel.velocity = theBallModel.estimate.velocity.rotated(theRobotPose.rotation);
  teammatesBallModel.isValid = true;
}

void OracledWorldModelProvider::update(GlobalOpponentsModel& globalOpponentsModel)
{
  computeRobotPose();
  globalOpponentsModel.opponents.clear();

  for(unsigned int i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
  {
    const Vector2f center(theRobotPose.inverse() * theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation);
    Vector2f left = center.normalized(Obstacle::getRobotDepth());
    Vector2f right = left;
    left.rotateLeft();
    right.rotateRight();
    left += center;
    right += center;

    if(center.squaredNorm() >= sqr(obstacleModelMaxDistance))
      continue;

    GlobalOpponentsModel::OpponentEstimate opponent;
    opponent.position = theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation;
    opponent.left = theRobotPose * left;
    opponent.right = theRobotPose * right;
    globalOpponentsModel.opponents.emplace_back(opponent);
  }
}

void OracledWorldModelProvider::update(ObstacleModel& obstacleModel)
{
  computeRobotPose();
  obstacleModel.obstacles.clear();

  auto toObstacle = [this, &obstacleModel](const GroundTruthWorldState::GroundTruthPlayer& player, bool isTeammate)
  {
    const Vector2f center(theRobotPose.inverse() * player.pose.translation);
    if(center.squaredNorm() >= sqr(obstacleModelMaxDistance))
      return;
    obstacleModel.obstacles.emplace_back(Matrix2f::Identity(), center, theFrameInfo.time,
                                         isTeammate ? (player.upright ? Obstacle::teammate : Obstacle::fallenTeammate)
                                                    : player.upright ? Obstacle::opponent : Obstacle::fallenOpponent);
    obstacleModel.obstacles.back().setLeftRight(Obstacle::getRobotDepth());
  };

  for(unsigned int i = 0; i < theGroundTruthWorldState.ownTeamPlayers.size(); ++i)
    toObstacle(theGroundTruthWorldState.ownTeamPlayers[i], true);
  for(unsigned int i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
    toObstacle(theGroundTruthWorldState.opponentTeamPlayers[i], false);
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
  static_cast<RobotPose&>(groundTruthRobotPose) = theRobotPose;
  groundTruthRobotPose.timestamp = theFrameInfo.time;
}

MAKE_MODULE(OracledWorldModelProvider);
