/**
 * @file SensorDataHistoryProvider.cpp
 * Implementation of module SensorDataHistoryProvider.
 * @author Philip Reichenberg
 */

#include "SensorDataHistoryProvider.h"
#include "Framework/Settings.h"

MAKE_MODULE(SensorDataHistoryProvider);

SensorDataHistoryProvider::SensorDataHistoryProvider()
{
  motionData.reserve(bufferSize);
}

void SensorDataHistoryProvider::update(SensorDataHistory& theSensorDataHistory)
{
  SensorDataHistory::SensorData data;
  data.jointAngles = theJointAngles;
  data.torsoMatrix = theTorsoMatrix;
  data.odometryData = theOdometryData;
  data.timestamp = theFrameInfo.time - delay;
  motionData.push_front(data);

  theSensorDataHistory.sensorData.clear();
  if(motionData.full())
  {
    for(std::size_t i = motionData.capacity() - 1; i < motionData.capacity(); i--)
      theSensorDataHistory.sensorData.push_back(motionData[i]);
  }
  else
    theSensorDataHistory.sensorData.push_back(data);
}
