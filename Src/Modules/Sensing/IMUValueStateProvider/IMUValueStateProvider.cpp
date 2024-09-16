/*
 * @file IMUValueStateProvider.cpp
 * @author Philip Reichenberg
 */

#include "IMUValueStateProvider.h"
#include "Debugging/Plot.h"
#include <cmath>

MAKE_MODULE(IMUValueStateProvider);

IMUValueStateProvider::IMUValueStateProvider()
{
  gyroValuesX.clear();
  gyroValuesY.clear();
  gyroValuesZ.clear();

  accValuesX.clear();
  accValuesY.clear();
  accValuesZ.clear();
}

void IMUValueStateProvider::update(IMUValueState& imuValueState)
{
  DECLARE_PLOT("module:IMUValueStateProvider:gyro:deviation:x");
  DECLARE_PLOT("module:IMUValueStateProvider:gyro:deviation:y");
  DECLARE_PLOT("module:IMUValueStateProvider:gyro:deviation:z");
  DECLARE_PLOT("module:IMUValueStateProvider:acc:deviation:x");
  DECLARE_PLOT("module:IMUValueStateProvider:acc:deviation:y");
  DECLARE_PLOT("module:IMUValueStateProvider:acc:deviation:z");

  imuValueState.filterTimeWindow = static_cast<int>(gyroValuesX.capacity() * Constants::motionCycleTime * 1000.f);

  // Sampling
  gyroValuesX.push_front(theRawInertialSensorData.gyro.x());
  gyroValuesY.push_front(theRawInertialSensorData.gyro.y());
  gyroValuesZ.push_front(theRawInertialSensorData.gyro.z());

  accValuesX.push_front(theRawInertialSensorData.acc.x());
  accValuesY.push_front(theRawInertialSensorData.acc.y());
  accValuesZ.push_front(theRawInertialSensorData.acc.z());

  // We did enough sampling
  if(gyroValuesX.full())
  {
    // calc gyro values
    // calc for x gyro
    imuValueState.gyroValues.mean.x() = gyroValuesX.average();
    imuValueState.gyroValues.deviation.x() = calcDeviation(imuValueState.gyroValues.mean.x(), gyroValuesX);
    // calc for y gyro
    imuValueState.gyroValues.mean.y() = gyroValuesY.average();
    imuValueState.gyroValues.deviation.y() = calcDeviation(imuValueState.gyroValues.mean.y(), gyroValuesY);
    // calc for z gyro
    imuValueState.gyroValues.mean.z() = gyroValuesZ.average();
    imuValueState.gyroValues.deviation.z() = calcDeviation(imuValueState.gyroValues.mean.z(), gyroValuesZ);

    // calc acc values
    // calc for x acc
    imuValueState.accValues.mean.x() = accValuesX.average();
    imuValueState.accValues.deviation.x() = calcDeviation(imuValueState.accValues.mean.x(), accValuesX);
    // calc for y acc
    imuValueState.accValues.mean.y() = accValuesY.average();
    imuValueState.accValues.deviation.y() = calcDeviation(imuValueState.accValues.mean.y(), accValuesY);
    // calc for z acc
    imuValueState.accValues.mean.z() = accValuesZ.average();
    imuValueState.accValues.deviation.z() = calcDeviation(imuValueState.accValues.mean.z(), accValuesZ);

    imuValueState.timestamp = theFrameInfo.time;

    PLOT("module:IMUValueStateProvider:gyro:deviation:x", Angle(imuValueState.gyroValues.deviation.x()).toDegrees());
    PLOT("module:IMUValueStateProvider:gyro:deviation:y", Angle(imuValueState.gyroValues.deviation.y()).toDegrees());
    PLOT("module:IMUValueStateProvider:gyro:deviation:z", Angle(imuValueState.gyroValues.deviation.z()).toDegrees());
    PLOT("module:IMUValueStateProvider:acc:deviation:x", imuValueState.accValues.deviation.x());
    PLOT("module:IMUValueStateProvider:acc:deviation:y", imuValueState.accValues.deviation.y());
    PLOT("module:IMUValueStateProvider:acc:deviation:z", imuValueState.accValues.deviation.z());

    // Calibrate gravity vector length
    if(theMotionInfo.executedPhase == MotionPhase::stand && theGroundContactState.contact &&
       imuValueState.accValues.deviation.x() < maxMeanAccDeviation &&
       imuValueState.accValues.deviation.y() < maxMeanAccDeviation &&
       imuValueState.accValues.deviation.z() < maxMeanAccDeviation)
    {
      accelerometerLengths.push_front(imuValueState.accValues.mean.norm());
      if(accelerometerLengths.full())
      {
        const float mean = accelerometerLengths.average();

        float meanSquaredDeviation = sqr(mean - accelerometerLengths[0]);
        float min = accelerometerLengths[0];
        float max = accelerometerLengths[0];
        for(size_t i = 1; i < accelerometerLengths.size(); ++i)
        {
          if(accelerometerLengths[i] < min)
            min = accelerometerLengths[i];
          else if(accelerometerLengths[i] > max)
            max = accelerometerLengths[i];
          meanSquaredDeviation += sqr(mean - accelerometerLengths[i]);
        }
        meanSquaredDeviation /= accelerometerLengths.size();

        if(meanSquaredDeviation < sqr(maxMeanAccDeviation) && (max - min) < 2.f * maxMeanAccDeviation)
          imuValueState.accLength = mean;
      }
    }
    else
      accelerometerLengths.clear();

    // robot is currently not standing still
    if(imuValueState.gyroValues.deviation.x() > thresholdGyroDeviation ||
       imuValueState.gyroValues.deviation.y() > thresholdGyroDeviation ||
       imuValueState.gyroValues.deviation.z() > thresholdGyroDeviation)
      imuValueState.gyroValues.deviationNotChangingSinceTimestamp = theFrameInfo.time;

    if(imuValueState.gyroValues.deviationNotChangingSinceTimestamp == theFrameInfo.time ||
       std::abs(imuValueState.gyroValues.mean.x()) > thresholdZero || // too much moving
       std::abs(imuValueState.gyroValues.mean.y()) > thresholdZero ||
       std::abs(imuValueState.gyroValues.mean.z()) > thresholdZero)
      imuValueState.gyroValues.stableSinceTimestamp = theFrameInfo.time;

    if(imuValueState.accValues.deviation.x() > thresholdAccDeviation ||
       imuValueState.accValues.deviation.y() > thresholdAccDeviation ||
       imuValueState.accValues.deviation.z() > thresholdAccDeviation)
      imuValueState.accValues.deviationNotChangingSinceTimestamp = theFrameInfo.time;

    if(imuValueState.accValues.deviationNotChangingSinceTimestamp == theFrameInfo.time ||
       std::abs(imuValueState.accValues.mean.norm() - imuValueState.accLength) > maxMeanAccDeviation)
      imuValueState.accValues.stableSinceTimestamp = theFrameInfo.time;

    if(imuValueState.gyroValues.stableSinceTimestamp == theFrameInfo.time || // too much deviation
       imuValueState.accValues.stableSinceTimestamp == theFrameInfo.time)
      imuValueState.notMovingSinceTimestamp = theFrameInfo.time;
  }
  else
  {
    imuValueState.notMovingSinceTimestamp = theFrameInfo.time;
    imuValueState.gyroValues.stableSinceTimestamp = theFrameInfo.time;
    imuValueState.accValues.stableSinceTimestamp = theFrameInfo.time;
    imuValueState.gyroValues.deviationNotChangingSinceTimestamp = theFrameInfo.time;
    imuValueState.accValues.deviationNotChangingSinceTimestamp = theFrameInfo.time;
  }
}

float IMUValueStateProvider::calcDeviation(const float mean, const RingBufferWithSum<float, 27>& buffer)
{
  float deviation = 0;
  for(const float angle : buffer)
  {
    deviation += sqr(angle - mean);
  }
  deviation /= static_cast<float>(buffer.size());
  deviation = std::sqrt(deviation);
  return deviation;
}
