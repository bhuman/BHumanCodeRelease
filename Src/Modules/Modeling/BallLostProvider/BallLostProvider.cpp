/**
 * @file BallLostProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "BallLostProvider.h"

MAKE_MODULE(BallLostProvider);

void BallLostProvider::update(BallLostModel& theBallLostModel)
{
  bool* lastFrameDetected = theCameraInfo.camera == CameraInfo::upper ? &lastUpperDetectedBall : &lastLowerDetectedBall;

  if(theFrameInfo.time != theBallModel.timeWhenLastSeen && theBallModel.timeOfLastCollision > lastBallSeenTimestamp)
    theBallLostModel.relativeAlternativBallPosition = theOdometer.odometryOffset.inverse() * theBallLostModel.relativeAlternativBallPosition;
  else if(theFrameInfo.time == theBallModel.timeWhenLastSeen)
  {
    if(!(*lastFrameDetected))
    {
      *lastFrameDetected = true;
      blockUpdate = true;
    }
    else
    {
      theBallLostModel.relativeAlternativBallPosition = theBallModel.estimate.position;
      theBallLostModel.relativeAlternativBallDirectionWhenLastSeen = theBallModel.estimate.position.angle();
      lastBallSeenTimestamp = theFrameInfo.time;
      blockUpdate = false;
    }
  }
  else if(!blockUpdate)
    theBallLostModel.relativeAlternativBallPosition = theBallModel.estimate.position;
}
