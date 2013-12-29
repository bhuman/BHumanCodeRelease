#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/FsrData.h"

#include "Tools/RingBufferWithSum.h"

MODULE(FsrDataProvider)
REQUIRES(RobotModel)
REQUIRES(SensorData)
PROVIDES_WITH_MODIFY(FsrData)
DEFINES_PARAMETER(float, fsrWeightOffset, 2.0f)
END_MODULE

/**
 * @class ExpGroundContactDetector
 * A module for sensor data filtering.
 */
class FsrDataProvider : public FsrDataProviderBase
{
public:
  /** Default constructor. */
  FsrDataProvider();

private:
  Vector2<> LFsrFL;
  Vector2<> LFsrFR;
  Vector2<> LFsrRL;
  Vector2<> LFsrRR;
  Vector2<> RFsrFL;
  Vector2<> RFsrFR;
  Vector2<> RFsrRL;
  Vector2<> RFsrRR;

  RingBufferWithSum<float, 10> fsrL;
  RingBufferWithSum<float, 10> fsrR;

  void update(FsrData& fsrData);
};
