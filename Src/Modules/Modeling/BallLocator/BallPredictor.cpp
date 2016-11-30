/**
 * @file BallPredictor.cpp
 * Declares a class that estimates the current position of the ball based on the most actual model.
 * @author Tim Laue
 */

#include "BallPredictor.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(BallPredictor, modeling)

void BallPredictor::update(BallPrediction& ballPrediction)
{
  // New Ball model?
  if(theBallModel.timeWhenLastSeen == timeOfLastExecution)
  {
    lastSeenBallModel = theBallModel;
    lastSeenBallModelOdometry = odometryOfLastFrame;
  }

  // Ball information is not "fresh"?
  if(theFrameInfo.getTimeSince(lastSeenBallModel.timeWhenLastSeen) <= validityTimeout)
  {
    // Physics update of rolling ball
    float deltaTime = static_cast<float>(theFrameInfo.time - lastSeenBallModel.timeWhenLastSeen) / 1000.f;
    Vector2f propagatedBallPosition = BallPhysics::propagateBallPosition(lastSeenBallModel.estimate.position, lastSeenBallModel.estimate.velocity,
                                                                         deltaTime, theFieldDimensions.ballFriction);
    // Robot motion
    Pose2f odometryOffset = lastSeenBallModelOdometry - theOdometryData;
    ballPrediction.position = odometryOffset * propagatedBallPosition;
    ballPrediction.isValid = true;
  }
  else
    ballPrediction.isValid = false;

  ballPrediction.timeWhenLastSeen = theBallModel.timeWhenLastSeen;
  timeOfLastExecution = theFrameInfo.time;
  odometryOfLastFrame = theOdometryData;
}
