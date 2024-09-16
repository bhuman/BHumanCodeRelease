/**
 * @file IMUValueStateProvider.h
 * This file declares a module that filters the gyro values and calculates the mean and the max deviation over a defined time.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(IMUValueStateProvider,
{,
  USES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(RawInertialSensorData),
  USES(MotionInfo),
  PROVIDES(IMUValueState),
  DEFINES_PARAMETERS(
  {,
    (Angle)(3_deg) thresholdGyroDeviation, // if the NAO is not moving, the gyros should vary max by this value. It must be high, because the gyros have a high deviation over a long time interval
    (Angle)(3_deg) thresholdZero, // if the NAO is not moving, the gyros should be lower than this value
    (float)(0.2f) thresholdAccDeviation,
    (float)(0.1f) maxMeanAccDeviation, /**< Max measured variance of multiple acc measurements, to calibrate the gravity vector. */
  }),
});

class IMUValueStateProvider : public IMUValueStateProviderBase
{
public:
  IMUValueStateProvider();
private:
  //RingBuffer for the last 27 gyro and acc values. So many values can be sampled in 333ms with the current motion time of 0.012ms.
  RingBufferWithSum<float, 27> gyroValuesX;
  RingBufferWithSum<float, 27> gyroValuesY;
  RingBufferWithSum<float, 27> gyroValuesZ;

  RingBufferWithSum<float, 27> accValuesX;
  RingBufferWithSum<float, 27> accValuesY;
  RingBufferWithSum<float, 27> accValuesZ;

  RingBufferWithSum<float, 50> accelerometerLengths;

  void update(IMUValueState& imuValueState) override;

  float calcDeviation(const float mean, const RingBufferWithSum<float, 27>& buffer);
};
