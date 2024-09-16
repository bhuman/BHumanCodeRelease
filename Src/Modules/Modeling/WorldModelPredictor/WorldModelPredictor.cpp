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

MAKE_MODULE(WorldModelPredictor);

void WorldModelPredictor::update(WorldModelPrediction& worldModelPrediction)
{
  // Predict robot pose:
  worldModelPrediction.robotPose = theRobotPose + theOdometer.odometryOffset;

  // Update kick time, if ball speed increased and ball was near.
  // This is necessary, as the ball model might have a changed velocity due to a kick but was not
  // seen after kicking. The model was changed by code for handling contacts.
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
  if(theGameState.isPenaltyShootout() &&
     theFrameInfo.getTimeSince(lastUsedBallModel.timeWhenLastSeen) > 100)
  {
    const Vector2f knownBallPosition = theGameState.isForOwnTeam()
      ? Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f)
      : Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
    worldModelPrediction.ballPosition = theRobotPose.inverse() * knownBallPosition;
    worldModelPrediction.ballVelocity = Vector2f::Zero();
  }

  worldModelPrediction.timeWhenBallLastSeen = theBallModel.timeWhenLastSeen;
  timeOfLastExecution = theFrameInfo.time;
  ballSpeedAtLastExecution = worldModelPrediction.ballVelocity.norm();
  odometryOfLastFrame = theOdometryData;
}
