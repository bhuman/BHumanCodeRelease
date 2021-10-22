/**
 * @file GyroStateProvider.h
 * This file declares a module that filters the gyro values and calculates the mean and the max deviation over a defined time.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Sensing/GyroState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(GyroStateProvider,
{,
  USES(FrameInfo),
  USES(InertialData),
  PROVIDES(GyroState),
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
  int samplingCounter;

  void update(GyroState& gyroOffset) override;
};
