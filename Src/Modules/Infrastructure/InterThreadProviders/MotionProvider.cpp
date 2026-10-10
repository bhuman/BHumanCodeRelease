/**
 * @file MotionProvider.cpp
 *
 * This file implements a module that provides representations from the motion
 * thread for the current frame.
 *
 * @author Philip Reichenberg
 * @author Thomas Röfer
 */

#include "MotionProvider.h"
#include <algorithm>

MAKE_MODULE(MotionProvider);

void MotionProvider::update(JointAngles& jointAngles)
{
  if(theSensorDataHistory.sensorData.empty())
  {
    motionData = nullptr;
    return;
  }
  else
  {
    const auto frame = std::lower_bound(theSensorDataHistory.sensorData.begin(), theSensorDataHistory.sensorData.end(), theFrameInfo.time,
                                        [](const SensorDataHistory::SensorData& sensorData, unsigned timestamp) {return sensorData.timestamp < timestamp;});
    if(frame != theSensorDataHistory.sensorData.end())
      motionData = &*frame;
    else
      motionData = &theSensorDataHistory.sensorData.back();
  }

  jointAngles = motionData->jointAngles;
}

void MotionProvider::update(OdometryData& odometryData)
{
  if(motionData)
    odometryData = motionData->odometryData;
}

void MotionProvider::update(RobotModel& robotModel)
{
  robotModel.setJointData(theJointAngles, theRobotDimensions, theMassCalibration);
}

void MotionProvider::update(TorsoMatrix& torsoMatrix)
{
  if(motionData)
    torsoMatrix = motionData->torsoMatrix;
}
