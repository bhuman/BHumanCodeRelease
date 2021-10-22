/*
 * @file GyroOffsetProvider.cpp
 * @author Philip Reichenberg
 */

#include "GyroOffsetProvider.h"
#include <cmath>

MAKE_MODULE(GyroOffsetProvider, infrastructure);

GyroOffsetProvider::GyroOffsetProvider()
{
  state = waiting;
  gyroMeanX.clear();
  gyroMeanY.clear();
  gyroMeanZ.clear();
  gyroDeviationX.clear();
  gyroDeviationY.clear();
  gyroDeviationZ.clear();
  gyroChecks = { 0, 0, 0 };
  timestamps = { 0, 0, 0 };
  samplingStart = startTimestamp;
  samplingCounter = 0;
  wasPlayingOnce = false;
  lastGyros = Vector3a::Zero();
  lastGyroChange = 0;
  gyroStuckTimestamp = 0;
  gyroStuckSoundTimestamp = 0;
}

void GyroOffsetProvider::update(GyroOffset& gyroOffset)
{
  if(SystemCall::getMode() != SystemCall::simulatedRobot)
    checkBodyDisconnection(gyroOffset);
  if(SystemCall::getMode() != SystemCall::physicalRobot)
    return;
  //Make sure the robot is not making sound when he is not in Initial.
  //I have no idea if this is enough in a case, where the game is already going on but the robot was rebooted.
  if(theGameInfo.state != STATE_INITIAL && wasPlayingOnce)
    return;
  if(theRobotInfo.penalty == 0 && (theGameInfo.state == STATE_PLAYING || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_READY))
  {
    wasPlayingOnce = true;
    return;
  }
  switch(state)
  {
    //waiting until the first time standHigh
    case waiting:
    {
      if(theGroundContactState.contact
         && ((theMotionRequest.motion == MotionRequest::stand && theMotionRequest.standHigh) || (theMotionRequest.motion == MotionRequest::playDead)))
      {
        //is the robot long enough in standhigh?
        if(theFrameInfo.getTimeSince(samplingStart) > waitTimeBeforeSampling)
        {
          state = sampling;
          samplingCounter = 0;
        }
      }
      else if(theFrameInfo.time != 0)
        samplingStart = theFrameInfo.time;
      break;
    }
    case sampling:
    {
      //if the motion request changed, we assume the robot starts playing now.
      if(!((theMotionRequest.motion == MotionRequest::stand && theMotionRequest.standHigh) || (theMotionRequest.motion == MotionRequest::playDead)))
      {
        state = waiting;
        break;
      }
      //when robot is picked up again, wait until he has ground contact
      if(!theGroundContactState.contact)
      {
        state = waiting;
        samplingCounter = 0;
        break;
      }
      //Sampling
      if(lastGyroStateUpdate != theGyroState.timestamp)
      {
        lastGyroStateUpdate = theGyroState.timestamp;
        gyroMeanX.push_front(theGyroState.mean.x());
        gyroMeanY.push_front(theGyroState.mean.y());
        gyroMeanZ.push_front(theGyroState.mean.z());
        gyroDeviationX.push_front(theGyroState.deviation.x());
        gyroDeviationY.push_front(theGyroState.deviation.y());
        gyroDeviationZ.push_front(theGyroState.deviation.z());
        samplingCounter += 1;
      }
      //We did enough sampling
      if(samplingCounter >= static_cast<int>(gyroMeanX.capacity()))
        state = set;
      break;
    }
    case set:
    {
      //check if for the windows of the last 333ms and the last 666-999ms, the gyro deviation was small enough
      if(gyroDeviationX[0] < thresholdGyroDeviation && gyroDeviationY[0] < thresholdGyroDeviation && gyroDeviationZ[0] < thresholdGyroDeviation // last 333ms
         && gyroDeviationX[2] < thresholdGyroDeviation && gyroDeviationY[2] < thresholdGyroDeviation && gyroDeviationZ[2] < thresholdGyroDeviation) // last 666-999ms
      {
        gyroOffset.offset.x() = std::abs(gyroMeanX[1]) > thresholdZero ? gyroMeanX[1] : 0_deg;
        gyroOffset.offset.y() = std::abs(gyroMeanY[1]) > thresholdZero ? gyroMeanY[1] : 0_deg;
        gyroOffset.offset.z() = std::abs(gyroMeanZ[1]) > thresholdZero ? gyroMeanZ[1] : 0_deg;
        if(std::abs(gyroMeanX[1]) > thresholdZero
           || std::abs(gyroMeanY[1]) > thresholdZero
           || std::abs(gyroMeanZ[1]) > thresholdZero)
        {
          float maxOffset = std::max(std::max(std::abs(gyroOffset.offset.x().toDegrees()), std::abs(gyroOffset.offset.y().toDegrees())), std::abs(gyroOffset.offset.z().toDegrees()));
          SystemCall::playSound("sirene.wav");
          SystemCall::say((std::string("Gyro has Offset ") + TypeRegistry::getEnumName(Global::getSettings().teamColor) + " " + std::to_string(theRobotInfo.number) + " with an offset of " +  std::to_string(static_cast<int>(maxOffset)) + " degrees").c_str());
          ANNOTATION("GyroOffsetProvider", "Added Offset " << gyroOffset.offset);
          OUTPUT_ERROR("GyroOffsetProvider - Added an Offset for the Gyros."); // Error, so we write it into the bhumand.log
        }
        state = off;
      }
      else
      {
        samplingCounter = -1;
        state = sampling;
      }
      break;
    }
    case off:
      break;
  }
}

void GyroOffsetProvider::checkBodyDisconnection(GyroOffset& gyroOffset)
{
  if(lastGyroChange == 0 || lastGyros != theInertialData.gyro)
  {
    if(gyroStuckTimestamp > theFrameInfo.time) // in case we are in a log file
      gyroStuckTimestamp = theFrameInfo.time - bodyDisconnectWaitTime;
    lastGyros = theInertialData.gyro,
    lastGyroChange = theFrameInfo.time;
    gyroOffset.bodyDisconnect = theFrameInfo.getTimeSince(gyroStuckTimestamp) < bodyDisconnectWaitTime;
  }
  else if(theFrameInfo.getTimeSince(lastGyroChange) > maxGyroDelay && lastGyroChange > startTimestamp)
  {
    gyroOffset.bodyDisconnect = true;
    gyroStuckTimestamp = theFrameInfo.time;
    if(SystemCall::getMode() == SystemCall::physicalRobot && theFrameInfo.getTimeSince(gyroStuckSoundTimestamp) > bodyDisconnectWaitTime)
    {
      gyroStuckSoundTimestamp = theFrameInfo.time;
      ANNOTATION("GyroOffsetProvider", "No body connection for " << theFrameInfo.getTimeSince(lastGyroChange) << "ms");
      SystemCall::playSound("sirene.wav");
      SystemCall::say((std::string("Body disconnect ") + TypeRegistry::getEnumName(Global::getSettings().teamColor) + " " + std::to_string(theRobotInfo.number)).c_str());
      OUTPUT_ERROR("Body Disconnect!");
    }
  }
}
