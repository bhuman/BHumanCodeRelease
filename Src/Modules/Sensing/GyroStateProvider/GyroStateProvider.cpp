/*
 * @file GyroStateProvider.cpp
 * @author Philip Reichenberg
 */

#include "GyroStateProvider.h"
#include "Debugging/Plot.h"
#include <cmath>

MAKE_MODULE(GyroStateProvider);

GyroStateProvider::GyroStateProvider()
{
  gyroValuesX.clear();
  gyroValuesY.clear();
  gyroValuesZ.clear();
}

void GyroStateProvider::update(GyroState& gyroState)
{
  DECLARE_PLOT("module:GyroStateProvider:deviation:x");
  DECLARE_PLOT("module:GyroStateProvider:deviation:y");
  DECLARE_PLOT("module:GyroStateProvider:deviation:z");

  gyroState.filterTimeWindow = static_cast<int>(gyroValuesX.capacity() * Constants::motionCycleTime * 1000.f);

  // TODO calculating also the highest single deviation would be good to, to detect transitions from "rest" to "moving" instantly
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

  // Sampling
  gyroValuesX.push_front(theInertialData.gyro.x());
  gyroValuesY.push_front(theInertialData.gyro.y());
  gyroValuesZ.push_front(theInertialData.gyro.z());

  // We did enough sampling
  if(gyroValuesX.full())
  {
    // calc for x gyro
    gyroState.mean.x() = gyroValuesX.average();
    gyroState.deviation.x() = calcDeviation(gyroState.mean.x(), gyroValuesX);
    // calc for y gyro
    gyroState.mean.y() = gyroValuesY.average();
    gyroState.deviation.y() = calcDeviation(gyroState.mean.y(), gyroValuesY);
    // calc for z gyro
    gyroState.mean.z() = gyroValuesZ.average();
    gyroState.deviation.z() = calcDeviation(gyroState.mean.z(), gyroValuesZ);
    gyroState.timestamp = theFrameInfo.time;

    PLOT("module:GyroStateProvider:deviation:x", gyroState.deviation.x().toDegrees());
    PLOT("module:GyroStateProvider:deviation:y", gyroState.deviation.y().toDegrees());
    PLOT("module:GyroStateProvider:deviation:z", gyroState.deviation.z().toDegrees());

    // robot is currently not standing still
    if(gyroState.deviation.x() > thresholdGyroDeviation ||
       gyroState.deviation.y() > thresholdGyroDeviation ||
       gyroState.deviation.z() > thresholdGyroDeviation)
      gyroState.gyroNotChangingSinceTimestamp = theFrameInfo.time;

    if(std::abs(gyroState.mean.x()) > thresholdZero ||
       std::abs(gyroState.mean.y()) > thresholdZero ||
       std::abs(gyroState.mean.z()) > thresholdZero ||
       gyroState.gyroNotChangingSinceTimestamp == theFrameInfo.time)
      gyroState.notMovingSinceTimestamp = theFrameInfo.time;
  }
  else
  {
    gyroState.notMovingSinceTimestamp = theFrameInfo.time;
    gyroState.gyroNotChangingSinceTimestamp = theFrameInfo.time;
  }
}
