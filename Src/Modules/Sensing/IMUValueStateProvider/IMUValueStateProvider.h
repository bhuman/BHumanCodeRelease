/**
 * @file IMUValueStateProvider.h
 * This file declares a module that filters the gyro values and calculates the mean and the max deviation over a defined time.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Sensing/InertialSensorData.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(IMUValueStateProvider,
{,
  USES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(InertialSensorData),
  USES(MotionInfo),
  PROVIDES(IMUValueState),
  LOADS_PARAMETERS(
  {,
    (Angle) thresholdGyroDeviation, // if the NAO is not moving, the gyros should vary max by this value. It must be high, because the gyros have a high deviation over a long time interval
    (Angle) thresholdZero, // if the NAO is not moving, the gyros should be lower than this value
    (float) thresholdAccDeviation,
    (float) maxMeanAccDeviation, /**< Max measured variance of multiple acc measurements, to calibrate the gravity vector. */
    (unsigned) bufferSize, /**< Size of the buffers for the imu values. Should be equal to about 324 ms. */
    (unsigned) accLengthBufferSize, /**< Sizeof the buffer for the acc length values. */
  }),
});

class IMUValueStateProvider : public IMUValueStateProviderBase
{
public:
  IMUValueStateProvider();
private:
  //RingBuffer of the imu values
  RingBufferWithSum<float> gyroValuesX;
  RingBufferWithSum<float> gyroValuesY;
  RingBufferWithSum<float> gyroValuesZ;

  RingBufferWithSum<float> accValuesX;
  RingBufferWithSum<float> accValuesY;
  RingBufferWithSum<float> accValuesZ;

  RingBufferWithSum<float> accelerometerLengths;

  void update(IMUValueState& imuValueState) override;

  float calcDeviation(const float mean, const RingBufferWithSum<float>& buffer);
};
