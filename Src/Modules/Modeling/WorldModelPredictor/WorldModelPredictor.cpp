/**
 * @file WorldModelPredictor.cpp
 *
 * This file implements a class that predicts the current state of data computed by modeling modules in the previous frame.
 * All positions receive odometry updates. If the ball was rolling, a dynamic update is performed, too.
 *
 * @author Tim Laue
 */

#include "WorldModelPredictor.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(WorldModelPredictor, modeling)

void WorldModelPredictor::update(WorldModelPrediction& worldModelPrediction)
{
  // Predict robot pose:
  worldModelPrediction.robotPose = theRobotPose + theOdometer.odometryOffset;

  // Update kick time, if ball speed increased and ball was near
  if((theBallModel.estimate.velocity.norm() > ballSpeedAtLastExecution) &&
     theBallModel.estimate.position.norm() < 400.f)
  {
    //OUTPUT(idText, text, "Ball was kicked! Current speed: " << theBallModel.estimate.velocity.norm() << "  Last speed: " << ballSpeedAtLastExecution);
    timeWhenLastKicked = timeOfLastExecution;
  }

  // New Ball model?
  if(theBallModel.timeWhenLastSeen == timeOfLastExecution || timeWhenLastKicked == timeOfLastExecution)
  {
    lastUsedBallModel = theBallModel;
    lastUsedBallModelOdometry = odometryOfLastFrame;
  }

  // Physics update of rolling ball
  unsigned int referenceTime = std::max(lastUsedBallModel.timeWhenLastSeen, timeWhenLastKicked);
  float deltaTime = static_cast<float>(theFrameInfo.time - referenceTime) / 1000.f;
  Vector2f propagatedBallPosition(lastUsedBallModel.estimate.position);
  Vector2f propagatedBallVelocity(lastUsedBallModel.estimate.velocity);
  BallPhysics::propagateBallPositionAndVelocity(propagatedBallPosition, propagatedBallVelocity,
                                                deltaTime, theBallSpecification.friction);

  // Incorporate robot motion
  Pose2f odometryOffset = lastUsedBallModelOdometry - theOdometryData;
  worldModelPrediction.ballPosition = odometryOffset * propagatedBallPosition;
  worldModelPrediction.ballVelocity = propagatedBallVelocity.rotate(odometryOffset.rotation);

  // Special handling for penalty shootout -> ball is supposed to be on the penalty spot!
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT &&
     theFrameInfo.getTimeSince(lastUsedBallModel.timeWhenLastSeen) > 100)
  {
    const Vector2f knownBallPosition = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber
      ? Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f)
      : Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
    worldModelPrediction.ballPosition = theRobotPose.inversePose * knownBallPosition;
    worldModelPrediction.ballVelocity = Vector2f::Zero();
    worldModelPrediction.ballIsPredictedByRule = true;
  }
  else
  {
    worldModelPrediction.ballIsPredictedByRule = false;
  }

  worldModelPrediction.timeWhenBallLastSeen = theBallModel.timeWhenLastSeen;
  timeOfLastExecution = theFrameInfo.time;
  ballSpeedAtLastExecution = worldModelPrediction.ballVelocity.norm();
  odometryOfLastFrame = theOdometryData;
}
