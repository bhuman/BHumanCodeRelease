/**
 * @file Modules/Infrastructure/MotionRobotHealthProvider.cpp
 * This file implements a module that provides information about the robot's health.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "MotionRobotHealthProvider.h"
#include "Platform/Time.h"

MAKE_MODULE(MotionRobotHealthProvider, infrastructure)

void MotionRobotHealthProvider::update(MotionRobotHealth& motionRobotHealth)
{
  // Compute frame rate of motion thread:
  unsigned now = Time::getCurrentSystemTime();
  if(lastExecutionTime != 0)
    timeBuffer.push_front(now - lastExecutionTime);
  motionRobotHealth.motionFrameRate = timeBuffer.sum() ? 1000.0f / timeBuffer.averagef() : 0.f;
  motionRobotHealth.avgMotionTime = float(timeBuffer.average());
  motionRobotHealth.maxMotionTime = float(timeBuffer.maximum());
  motionRobotHealth.minMotionTime = float(timeBuffer.minimum());
  lastExecutionTime = now;
}
