/**
 * @file GyroStateProvider.h
 * This file declares a module that filters the gyro values and calculates the mean and the max deviation over a defined time.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Sensing/InertialData.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"

MODULE(GyroStateProvider,
{,
  USES(FrameInfo),
  USES(InertialData),
  PROVIDES(GyroState),
  DEFINES_PARAMETERS(
  {,
    (Angle)(3_deg) thresholdGyroDeviation, // if the NAO is not moving, the gyros should vary max by this value. It must be high, because the gyros have a high deviation over a long time interval
    (Angle)(3_deg) thresholdZero, // if the NAO is not moving, the gyros should be lower than this value
  }),
});

class GyroStateProvider : public GyroStateProviderBase
{
public:
  GyroStateProvider();
private:
  //RingBuffer for the last 27 gyro values. So many values can be sampled in 333ms with the current motion time of 0.012ms.
  RingBufferWithSum<Angle, 27> gyroValuesX;
  RingBufferWithSum<Angle, 27> gyroValuesY;
  RingBufferWithSum<Angle, 27> gyroValuesZ;

  void update(GyroState& gyroOffset) override;
};
