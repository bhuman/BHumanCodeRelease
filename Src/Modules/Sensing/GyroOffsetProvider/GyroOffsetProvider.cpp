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
  gyroOffset.offsetCheckFinished = true;
}

void GyroOffsetProvider::checkBodyDisconnection(GyroOffset& gyroOffset)
{
  if(lastGyroChange == 0 || lastGyros != theInertialSensorData.gyro
     || (bodyDisconnectGyroRange.isInside(theInertialSensorData.gyro.x()) &&
         bodyDisconnectGyroRange.isInside(theInertialSensorData.gyro.y()) &&
         bodyDisconnectGyroRange.isInside(theInertialSensorData.gyro.z())))
  {
    if(gyroStuckTimestamp > theFrameInfo.time) // in case we are in a log file
      gyroStuckTimestamp = theFrameInfo.time - bodyDisconnectWaitTime;
    lastGyros = theInertialSensorData.gyro,
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
        SystemCall::playSound("siren.wav", true);
        SystemCall::say((std::string("GyroOffsetProvider Body disconnect ") + TypeRegistry::getEnumName(theGameState.color()) + " " + std::to_string(theGameState.playerNumber)).c_str(), true);
        OUTPUT_ERROR("GyroOffsetProvider Body Disconnect!");
      }
    }
  }
}
