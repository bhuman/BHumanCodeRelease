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
    theBallLostModel.relativeAlternateBallPosition = theOdometer.odometryOffset.inverse() * theBallLostModel.relativeAlternateBallPosition;
  else if(theFrameInfo.time == theBallModel.timeWhenLastSeen)
  {
    if(!(*lastFrameDetected))
    {
      *lastFrameDetected = true;
      blockUpdate = true;
    }
    else
    {
      theBallLostModel.relativeAlternateBallPosition = theBallModel.estimate.position;
      theBallLostModel.relativeAlternateBallDirectionWhenLastSeen = theBallModel.estimate.position.angle();
      lastBallSeenTimestamp = theFrameInfo.time;
      blockUpdate = false;
    }
  }
  else if(!blockUpdate)
    theBallLostModel.relativeAlternateBallPosition = theBallModel.estimate.position;
}
