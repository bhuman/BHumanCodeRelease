/**
 * @file Modules/Infrastructure/SimulatedWorldModelProvider.cpp
 *
 * This file implements a module that provides models based on simulated data.
 *
 * @author Tim Laue
 */

#include "SimulatedWorldModelProvider.h"
#include "Streaming/Global.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Math/Pose3f.h"
#include "Tools/Math/Projection.h"
#include "Tools/Modeling/BallPhysics.h"

SimulatedWorldModelProvider::SimulatedWorldModelProvider():
  lastBallModelComputation(0), lastBallVelocityUpdate(0), lastGroundTruthBallModelComputation(0), lastRobotPoseComputation(0)
{}

void SimulatedWorldModelProvider::computeRobotPose()
{
  if(lastRobotPoseComputation == theFrameInfo.time)
    return;
  theRobotPose = theGroundTruthWorldState.ownPose;
  theRobotPose.quality = RobotPose::superb;
  lastRobotPoseComputation = theFrameInfo.time;
}

bool SimulatedWorldModelProvider::ballIsVisible(const Vector2f& ballPosition)
{
  // Ball is too far away?
  if(ballPosition.norm() > ballModelMaxPerceptionDistance)
    return false;
  // Ball is (at least partially) in current camera image?
  Geometry::Circle circle;
  if(Projection::calculateBallInImage(ballPosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, circle))
  {
    const float projectionThreshold = circle.radius / 1.5f;
    if((circle.center.x() >= -projectionThreshold) &&
       (circle.center.x() < theCameraInfo.width + projectionThreshold) &&
       (circle.center.y() >= -projectionThreshold) &&
       (circle.center.y() < theCameraInfo.height + projectionThreshold))
    {
      return true;
    }
  }
  // TODO: Maybe consider shoulders / occlusion by other robots
  return false;
}

void SimulatedWorldModelProvider::updateInternalBallModel()
{
  computeRobotPose();
  const Vector2f ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
  const Vector2f ballVelocity = theGroundTruthWorldState.balls[0].velocity.head<2>();
  const Vector2f relativeBallPosition = theRobotPose.inverse() * ballPosition;
  if(perfectBallModel || ballIsVisible(relativeBallPosition))
  {
    theBallModel.estimate.position = relativeBallPosition;
    theBallModel.estimate.velocity = ballVelocity.rotated(-theRobotPose.rotation);
    theBallModel.estimate.radius = theBallSpecification.radius;
    theBallModel.lastPerception = theBallModel.estimate.position;
    theBallModel.timeWhenLastSeen = theFrameInfo.time;
    theBallModel.timeWhenDisappeared = theFrameInfo.time;
    lastBallModelComputation = theFrameInfo.time;
    lastBallVelocityUpdate = theFrameInfo.time;
    timeWhenBallFirstDisappeared = theFrameInfo.time;
    ballNotSeenButShouldBeSeenCounter = 0;
  }
  // Ball is not seen, but ...
  else
  {
    // ... the robot has moved:
    const Pose2f odometryOffset = theOdometer.odometryOffset.inverse();
    theBallModel.lastPerception = odometryOffset * theBallModel.lastPerception;
    theBallModel.estimate.position  = odometryOffset * theBallModel.estimate.position;
    theBallModel.estimate.velocity.rotate(odometryOffset.rotation);
    // ... it was moving when it was seen the last time:
    if(theBallModel.estimate.velocity.squaredNorm() != 0.f)
    {
      int deltaTimeMilliSeconds = theFrameInfo.getTimeSince(lastBallVelocityUpdate);
      if(deltaTimeMilliSeconds != 0)
      {
        float deltaTimeSeconds = deltaTimeMilliSeconds / 1000.f;
        BallPhysics::propagateBallPositionAndVelocity(theBallModel.estimate.position, theBallModel.estimate.velocity,
                                                      deltaTimeSeconds, theBallSpecification.friction);
        lastBallVelocityUpdate = theFrameInfo.time;
      }
    }
    bool ballShouldBeVisible = ballIsVisible(theBallModel.estimate.position);
    if(ballShouldBeVisible)
    {
      ++ballNotSeenButShouldBeSeenCounter;
    }
    else if(!ballNotSeenButShouldBeSeenCounter)
    {
      timeWhenBallFirstDisappeared = theFrameInfo.time;
    }
    if(ballNotSeenButShouldBeSeenCounter >= ballDisappearedThreshold)
      theBallModel.timeWhenDisappeared = timeWhenBallFirstDisappeared;
    else
      theBallModel.timeWhenDisappeared = theFrameInfo.time;
  }
}

void SimulatedWorldModelProvider::update(BallModel& ballModel)
{
  if(lastBallModelComputation != theFrameInfo.time && theGroundTruthWorldState.balls.size() > 0)
    updateInternalBallModel();
  ballModel = theBallModel;
}

void SimulatedWorldModelProvider::update(GroundTruthBallModel& groundTruthBallModel)
{
  if(lastGroundTruthBallModelComputation == theFrameInfo.time || theGroundTruthWorldState.balls.size() == 0)
    return;
  const Vector2f ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
  const Vector2f ballVelocity = theGroundTruthWorldState.balls[0].velocity.head<2>();
  groundTruthBallModel.estimate.position = theGroundTruthWorldState.ownPose.inverse() * ballPosition;
  groundTruthBallModel.estimate.velocity = ballVelocity.rotated(-theGroundTruthWorldState.ownPose.rotation);
  groundTruthBallModel.lastPerception = groundTruthBallModel.estimate.position;
  groundTruthBallModel.timeWhenLastSeen = theFrameInfo.time;
  groundTruthBallModel.timeWhenDisappeared = theFrameInfo.time;
  lastGroundTruthBallModelComputation = theFrameInfo.time;
}

void SimulatedWorldModelProvider::update(GlobalOpponentsModel& globalOpponentsModel)
{
  computeRobotPose();
  globalOpponentsModel.opponents.clear();

  for(unsigned int i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
  {
    const Vector2f center(theRobotPose.inverse() * theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation);
    Vector2f left = center.normalized(theRobotDimensions.robotDepth);
    Vector2f right = left;
    left.rotateLeft();
    right.rotateRight();
    left += center;
    right += center;

    if(center.squaredNorm() >= sqr(obstacleModelMaxPerceptionDistance))
      continue;

    GlobalOpponentsModel::OpponentEstimate opponent;
    opponent.position = theGroundTruthWorldState.opponentTeamPlayers[i].pose.translation;
    opponent.left = theRobotPose * left;
    opponent.right = theRobotPose * right;
    globalOpponentsModel.opponents.emplace_back(opponent);
  }
}

void SimulatedWorldModelProvider::update(ObstacleModel& obstacleModel)
{
  computeRobotPose();
  obstacleModel.obstacles.clear();

  auto toObstacle = [this, &obstacleModel](const GroundTruthWorldState::GroundTruthPlayer& player, bool isTeammate)
  {
    const Vector2f center(theRobotPose.inverse() * player.pose.translation);
    if(center.squaredNorm() >= sqr(obstacleModelMaxPerceptionDistance))
      return;
    obstacleModel.obstacles.emplace_back(Matrix2f::Identity(), center, theRobotDimensions.robotDepth, theFrameInfo.time,
                                         isTeammate ? (player.upright ? Obstacle::teammate : Obstacle::fallenTeammate)
                                                    : player.upright ? Obstacle::opponent : Obstacle::fallenOpponent);
    obstacleModel.obstacles.back().setLeftRight(theRobotDimensions.robotDepth);
  };

  for(unsigned int i = 0; i < theGroundTruthWorldState.ownTeamPlayers.size(); ++i)
    toObstacle(theGroundTruthWorldState.ownTeamPlayers[i], true);
  for(unsigned int i = 0; i < theGroundTruthWorldState.opponentTeamPlayers.size(); ++i)
    toObstacle(theGroundTruthWorldState.opponentTeamPlayers[i], false);
}

void SimulatedWorldModelProvider::update(RobotPose& robotPose)
{
  computeRobotPose();
  robotPose = theRobotPose;
}

void SimulatedWorldModelProvider::update(GroundTruthRobotPose& groundTruthRobotPose)
{
  computeRobotPose();
  static_cast<RobotPose&>(groundTruthRobotPose) = theRobotPose;
  groundTruthRobotPose.timestamp = theFrameInfo.time;
}

MAKE_MODULE(SimulatedWorldModelProvider);
