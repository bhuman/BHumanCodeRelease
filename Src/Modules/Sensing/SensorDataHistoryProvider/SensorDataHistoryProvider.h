/**
 * @file SensorDataHistoryProvider.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/SensorDataHistory.h"

MODULE(SensorDataHistoryProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(OdometryData),
  REQUIRES(TorsoMatrix),
  PROVIDES(SensorDataHistory),
  LOADS_PARAMETERS(
  {,
    (int) delay, /**< Extra delay between sensor data and perception. */
    (unsigned) bufferSize, /**< History size. */
  }),
});

class SensorDataHistoryProvider : public SensorDataHistoryProviderBase
{
public:
  SensorDataHistoryProvider();
private:
  RingBuffer<SensorDataHistory::SensorData, 1> motionData;
  void update(SensorDataHistory& theSensorDataHistory) override;
};
