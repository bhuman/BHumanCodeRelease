/**
* @file Modules/Infrastructure/MotionRobotHealthProvider.h
* This file implements a module that provides information about the robot's health.
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#include "MotionRobotHealthProvider.h"

void MotionRobotHealthProvider::update(MotionRobotHealth& motionRobotHealth)
{
  // Compute frame rate of motion process:
  unsigned now = SystemCall::getCurrentSystemTime();
  if(lastExecutionTime != 0)
    timeBuffer.add(now - lastExecutionTime);
  motionRobotHealth.motionFrameRate = timeBuffer.getSum() ? 1000.0f / (static_cast<float>(timeBuffer.getSum()) / timeBuffer.getNumberOfEntries()) : 0.0f;
  motionRobotHealth.avgMotionTime = float(timeBuffer.getAverage());
  motionRobotHealth.maxMotionTime = float(timeBuffer.getMaximum());
  motionRobotHealth.minMotionTime = float(timeBuffer.getMinimum());
  lastExecutionTime = now;
}

MAKE_MODULE(MotionRobotHealthProvider, Motion Infrastructure)
