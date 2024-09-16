/**
 * @file Modules/Infrastructure/MotionRobotHealthProvider.cpp
 * This file implements a module that provides information about the robot's health.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Yannik Meinken
 */

#include "Debugging/Annotation.h"
#include "MotionRobotHealthProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"

MAKE_MODULE(MotionRobotHealthProvider);

void MotionRobotHealthProvider::update(MotionRobotHealth& motionRobotHealth)
{
  // Compute frame rate of motion thread:
  unsigned now = Time::getCurrentSystemTime();
  if(lastExecutionTime != 0)
    timeBuffer.push_front(now - lastExecutionTime);
  lastExecutionTime = now;
  motionRobotHealth.motionFrameRate = timeBuffer.sum() ? 1000.0f / timeBuffer.averagef() : 0.f;
  motionRobotHealth.avgMotionTime = float(timeBuffer.average());
  motionRobotHealth.maxMotionTime = float(timeBuffer.maximum());
  motionRobotHealth.minMotionTime = float(timeBuffer.minimum());

  // In logs the time can jump backwards
  if(lastFrameTime > theFrameInfo.time)
    lastFrameTime = theFrameInfo.time;
  if(lastFrameTime != 0)
  {
    const unsigned deltaT = theFrameInfo.getTimeSince(lastFrameTime);
    const bool frameDroppedDueToTime = deltaT > droppedFactorThreshold * Constants::motionCycleTime * 1000;

    // if both the acc and gyro data changed outside simulation we must have missed a frame
    const bool frameDroppedDueToSensorData = theRawInertialSensorData.acc != lastInertialSensorData.acc &&
                                             theRawInertialSensorData.gyro != lastInertialSensorData.gyro &&
                                             SystemCall::getMode() != SystemCall::Mode::simulatedRobot;
    lastInertialSensorData = theRawInertialSensorData;
    if(frameDroppedDueToSensorData != frameDroppedDueToTime)
      ANNOTATION("MotionRobotHealthProvider", "Missing motionframe detected by only one measure"); //Don't want to assert because it depends on parametrization

    // minus one because the normal frame time is included
    // max to prevent overflow and to get 1 even if the time was right but a dropped frame was detected by the sensor data
    const unsigned numOfDroppedFrames = std::max(static_cast<unsigned>(std::round(deltaT / (Constants::motionCycleTime * 1000.f))), 2u) - 1u;
    motionRobotHealth.motionFramesDropped = numOfDroppedFrames *
                                            (frameDroppedDueToSensorData || frameDroppedDueToTime); // Multiplication to get zero in case no frame was dropped

    motionRobotHealth.frameLostStatus = MotionRobotHealth::noFrameLost;

    if(motionRobotHealth.motionFramesDropped)
    {
      motionRobotHealth.frameLostStatus = MotionRobotHealth::oneFrameLost;
      if(motionRobotHealth.motionFramesDropped >= multipleFramesDroppedThreshold)
        motionRobotHealth.frameLostStatus = MotionRobotHealth::multipleFramesLost;
      if(motionRobotHealth.motionFramesDropped * Constants::motionCycleTime > bodyDisconnectThreshold)
      {
        motionRobotHealth.frameLostStatus = MotionRobotHealth::bodyDisconnect;
        if(SystemCall::getMode() == SystemCall::physicalRobot && theFrameInfo.getTimeSince(bodyDisconnectTimestamp) > bodyDisconnectWarningTime &&
           executionTimes > executionTimesThreshold)
        {
          bodyDisconnectTimestamp = theFrameInfo.time;
          ANNOTATION("MotionRobotHealthProvider", "No body connection for " << theFrameInfo.getTimeSince(lastFrameTime) << "ms");
          SystemCall::playSound("sirene.wav", true);
          SystemCall::say(
            (std::string("Body disconnect ") + TypeRegistry::getEnumName(theGameState.color()) + " " + std::to_string(theGameState.playerNumber)).c_str(),
            true);
          OUTPUT_ERROR("Body Disconnect!");
        }
      }
    }
  }
  lastFrameTime = theFrameInfo.time;
  executionTimes += !(executionTimes > executionTimesThreshold);
}
