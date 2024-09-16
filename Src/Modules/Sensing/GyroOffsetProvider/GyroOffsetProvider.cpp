/*
 * @file GyroOffsetProvider.cpp
 * @author Philip Reichenberg
 */

#include "GyroOffsetProvider.h"
#include "Debugging/Annotation.h"
#include "Platform/SystemCall.h"
#include <cmath>

MAKE_MODULE(GyroOffsetProvider);

GyroOffsetProvider::GyroOffsetProvider()
{
  samplingStart = startTimestamp;
}

void GyroOffsetProvider::update(GyroOffset& gyroOffset)
{
  if(SystemCall::getMode() != SystemCall::simulatedRobot)
    checkBodyDisconnection(gyroOffset);
  if(SystemCall::getMode() != SystemCall::physicalRobot)
  {
    gyroOffset.offsetCheckFinished = true;
    return;
  }
  // Gyro has offsets. Say that the robot needs a reboot.
  if(gyroOffset.isIMUBad && theFrameInfo.getTimeSince(gyroOffsetSoundTimestamp) > gyroOffsetWarningTime)
  {
    const Angle maxOffset = gyroOffset.offset.maxCoeff();
    gyroOffsetSoundTimestamp = theFrameInfo.time;
    SystemCall::playSound("sirene.wav", true);
    SystemCall::say((std::string("Please reboot me. My gyros have an offset of ") + std::to_string(static_cast<int>(maxOffset.toDegrees())) + " degrees!").c_str(), true);
  }
  if(gyroOffset.offsetCheckFinished)
    return;

  if(timeStampStart == 0 && theFrameInfo.time != 0)
    timeStampStart = theFrameInfo.time;
  if((timeStampStart != 0 && theFrameInfo.getTimeSince(timeStampStart) > waitForInitialGroundContactTime) ||
     theGameState.playerState != GameState::unstiff)
    hadGroundContactOnce |= theGroundContactState.contact || theGameState.playerState != GameState::unstiff;

  if(!hadGroundContactOnce)
    return;

  switch(state)
  {
    case waiting:
    {
      if(theGroundContactState.contact && theMotionInfo.executedPhase == MotionPhase::playDead)
      {
        // is the robot long enough in standhigh?
        if(theFrameInfo.getTimeSince(samplingStart) > waitTimeBeforeSampling)
        {
          state = sampling;
          gyroMeanX.clear();
          gyroMeanY.clear();
          gyroMeanZ.clear();
        }
      }
      else if(theFrameInfo.time != 0)
      {
        samplingStart = theFrameInfo.time;
        if(theFrameInfo.getTimeSince(gyroNotCheckedTimestamp) > gyroNotCheckedWarningTime)
        {
          gyroNotCheckedTimestamp = theFrameInfo.time;
          SystemCall::say("Please place me on the ground and do not move me. I need to calibrate my gyro values!", true);
        }
      }
      break;
    }
    case sampling:
    {
      // when robot is picked up again, wait until he has ground contact
      if(!theGroundContactState.contact)
      {
        state = waiting;
        break;
      }
      // Sampling
      if(lastGyroStateUpdate + theIMUValueState.filterTimeWindow < theIMUValueState.timestamp)
      {
        lastGyroStateUpdate = theIMUValueState.timestamp;
        gyroMeanX.push_front(theIMUValueState.gyroValues.mean.x());
        gyroMeanY.push_front(theIMUValueState.gyroValues.mean.y());
        gyroMeanZ.push_front(theIMUValueState.gyroValues.mean.z());
      }
      // We did enough sampling
      if(gyroMeanX.full())
        state = set;
      break;
    }
    case set:
    {
      // check if for the windows of the last 333ms and the last 666-999ms, the gyro deviation was small enough
      // use filer window * 3 + some safe offset, in case sensor data was lost
      if(theFrameInfo.getTimeSince(theIMUValueState.gyroValues.deviationNotChangingSinceTimestamp) > theIMUValueState.filterTimeWindow * 3 + 100)
      {
        gyroOffset.offset.x() = std::abs(gyroMeanX[1]) > thresholdZero ? std::abs(gyroMeanX[1]) : 0;
        gyroOffset.offset.y() = std::abs(gyroMeanY[1]) > thresholdZero ? std::abs(gyroMeanY[1]) : 0;
        gyroOffset.offset.z() = std::abs(gyroMeanZ[1]) > thresholdZero ? std::abs(gyroMeanZ[1]) : 0;
        if(gyroOffset.offset.x() > thresholdZero
           || gyroOffset.offset.y() > thresholdZero
           || gyroOffset.offset.z() > thresholdZero)
        {
          const Angle maxOffset = gyroOffset.offset.maxCoeff();
          SystemCall::playSound("sirene.wav", true);
          SystemCall::say((std::string("Gyro has Offset ") + TypeRegistry::getEnumName(theGameState.color()) + " " + std::to_string(theGameState.playerNumber) + " with an offset of " + std::to_string(static_cast<int>(maxOffset.toDegrees())) + " degrees").c_str(), true);
          ANNOTATION("GyroOffsetProvider", "Added Offset " << gyroOffset.offset);
          OUTPUT_ERROR("GyroOffsetProvider - Added an Offset for the Gyros."); // Error, so we write it into the bhumand.log
          gyroOffsetSoundTimestamp = theFrameInfo.time;
          gyroOffset.isIMUBad = true;
        }
        gyroOffset.offsetCheckFinished = true;
        if(gyroNotCheckedTimestamp != 0) // Only say it, if robot needed to say the warning
        {
          SystemCall::say("Thank you. I am ready to be used.", true);
        }
        state = off;
      }
      else
      {
        gyroMeanX.clear();
        gyroMeanY.clear();
        gyroMeanZ.clear();
        state = sampling;
        if(theFrameInfo.getTimeSince(gyroNotCheckedTimestamp) > gyroNotCheckedWarningTime)
        {
          gyroNotCheckedTimestamp = theFrameInfo.time;
          SystemCall::say("Please place me on the ground and do not move me. I need to calibrate my gyro values!", true);
        }
      }
      break;
    }
    case off:
      break;
  }
}

void GyroOffsetProvider::checkBodyDisconnection(GyroOffset& gyroOffset)
{
  if(lastGyroChange == 0 || lastGyros != theRawInertialSensorData.gyro
     || (bodyDisconnectGyroRange.isInside(theRawInertialSensorData.gyro.x()) &&
         bodyDisconnectGyroRange.isInside(theRawInertialSensorData.gyro.y()) &&
         bodyDisconnectGyroRange.isInside(theRawInertialSensorData.gyro.z())))
  {
    if(gyroStuckTimestamp > theFrameInfo.time) // in case we are in a log file
      gyroStuckTimestamp = theFrameInfo.time - bodyDisconnectWaitTime;
    lastGyros = theRawInertialSensorData.gyro,
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
      // This detection should no longer be necessary. TODO Delete it after last competition 2025?
      if(theMotionRobotHealth.frameLostStatus != RobotHealth::bodyDisconnect)
      {
        ANNOTATION("GyroOffsetProvider", "Module detected a body disconnect that MotionRobotHealth did not detect!");
        SystemCall::playSound("sirene.wav", true);
        SystemCall::say((std::string("GyroOffsetProvider Body disconnect ") + TypeRegistry::getEnumName(theGameState.color()) + " " + std::to_string(theGameState.playerNumber)).c_str(), true);
        OUTPUT_ERROR("GyroOffsetProvider Body Disconnect!");
      }
    }
  }
}
