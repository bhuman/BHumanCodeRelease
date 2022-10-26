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
  DECLARE_PLOT("module:GyroStateProvider:deviation:x");
  DECLARE_PLOT("module:GyroStateProvider:deviation:y");
  DECLARE_PLOT("module:GyroStateProvider:deviation:z");

  auto calcDeviation = [](const Angle& mean, const RingBufferWithSum<Angle, 27>& buffer)
  {
    Angle deviation = 0;
    for(const Angle& angle : buffer)
    {
      deviation += sqr(angle - mean);
    }
    deviation /= static_cast<float>(buffer.size());
    deviation = std::sqrt(deviation);
    return deviation;
  };

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
    gyroState.deviation.x() = calcDeviation(gyroState.mean.x(), gyroValuesX);
    //calc for y gyro
    gyroState.mean.y() = gyroValuesY.average();
    gyroState.deviation.y() = calcDeviation(gyroState.mean.y(), gyroValuesY);
    //calc for z gyro
    gyroState.mean.z() = gyroValuesZ.average();
    gyroState.deviation.z() = calcDeviation(gyroState.mean.z(), gyroValuesZ);
    gyroState.timestamp = theFrameInfo.time;

    PLOT("module:GyroStateProvider:deviation:x", gyroState.deviation.x().toDegrees());
    PLOT("module:GyroStateProvider:deviation:y", gyroState.deviation.y().toDegrees());
    PLOT("module:GyroStateProvider:deviation:z", gyroState.deviation.z().toDegrees());
    isNotMoving = std::abs(gyroState.mean.x()) < thresholdZero && std::abs(gyroState.mean.y()) < thresholdZero && std::abs(gyroState.mean.z()) < thresholdZero
                  && gyroState.deviation.x() < thresholdGyroDeviation && gyroState.deviation.y() < thresholdGyroDeviation && gyroState.deviation.z() < thresholdGyroDeviation;
  }

  if(!isNotMoving)
    gyroState.notMovingSinceTimestamp = theFrameInfo.time;
}
