/**
 * @file WorldModelPredictor.h
 *
 * This file implements a class that predicts the current state of data computed by modeling modules in the previous frame .
 * All positions have receive odometry updates. If the ball was rolling, a dynamic update is been performed, too.
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

  // New Ball model?
  if(theBallModel.timeWhenLastSeen == timeOfLastExecution)
  {
    lastSeenBallModel = theBallModel;
    lastSeenBallModelOdometry = odometryOfLastFrame;
  }

  // Ball information is not "fresh"?
  if(theFrameInfo.getTimeSince(lastSeenBallModel.timeWhenLastSeen) <= ballValidityTimeout)
  {
    // Physics update of rolling ball
    float deltaTime = static_cast<float>(theFrameInfo.time - lastSeenBallModel.timeWhenLastSeen) / 1000.f;
    Vector2f propagatedBallPosition = BallPhysics::propagateBallPosition(lastSeenBallModel.estimate.position, lastSeenBallModel.estimate.velocity,
                                                                         deltaTime, theBallSpecification.friction);
    // Robot motion
    Pose2f odometryOffset = lastSeenBallModelOdometry - theOdometryData;
    worldModelPrediction.ballPosition = odometryOffset * propagatedBallPosition;
    worldModelPrediction.ballIsValid = true;
  }
  else if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
  {
    worldModelPrediction.ballPosition = theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0);
    worldModelPrediction.ballIsValid = true;
  }
  else
  {
    worldModelPrediction.ballIsValid = false;
  }
  worldModelPrediction.timeWhenBallLastSeen = theBallModel.timeWhenLastSeen;
  timeOfLastExecution = theFrameInfo.time;
  odometryOfLastFrame = theOdometryData;
}
