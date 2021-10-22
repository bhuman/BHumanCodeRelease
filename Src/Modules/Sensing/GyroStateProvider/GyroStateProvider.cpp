/*
 * @file GyroStateProvider.cpp
 * @author Philip Reichenberg
 */

#include "GyroStateProvider.h"
#include <cmath>

MAKE_MODULE(GyroStateProvider, infrastructure);

GyroStateProvider::GyroStateProvider()
{
  gyroValuesX.clear();
  gyroValuesY.clear();
  gyroValuesZ.clear();
  samplingCounter = 0;
}

void GyroStateProvider::update(GyroState& gyroState)
{
  //Sampling
  gyroValuesX.push_front(theInertialData.gyro.x());
  gyroValuesY.push_front(theInertialData.gyro.y());
  gyroValuesZ.push_front(theInertialData.gyro.z());
  samplingCounter += 1;
  //We did enough sampling
  if(samplingCounter >= static_cast<int>(gyroValuesX.capacity()))
  {
    samplingCounter = 0;
    //calc for x gyro
    gyroState.mean.x() = gyroValuesX.average();
    gyroState.deviation.x() = std::abs(gyroValuesX.maximum() - gyroValuesX.minimum());
    //calc for y gyro
    gyroState.mean.y() = gyroValuesY.average();
    gyroState.deviation.y() = std::abs(gyroValuesY.maximum() - gyroValuesY.minimum());
    //calc for z gyro
    gyroState.mean.z() = gyroValuesZ.average();
    gyroState.deviation.z() = std::abs(gyroValuesZ.maximum() - gyroValuesZ.minimum());
    gyroState.timestamp = theFrameInfo.time;
  }
}
