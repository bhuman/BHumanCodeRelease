/**
 * @file StandBodyRotationProvider.h
 * Declaration a module that provides StandBodyRotation
 * @author jesse
 */

#pragma once

#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/StandBodyRotation.h"
#include "Tools/Module/Module.h"

MODULE(StandBodyRotationProvider,
{,
  REQUIRES(MotionRequest),
  REQUIRES(JointSensorData),
  REQUIRES(MotionSelection),
  PROVIDES(StandBodyRotation),
  LOADS_PARAMETERS(
  {,
    (Angle)(0.2f) maxXRotation, ///< the maximum of posible rotation / note: its the double of the maximum roation to one side
    (unsigned)(12) amountToAverage, ///< the amount of data, which is used to average to one data
    (unsigned)(120) minStepsToFullRoation, ///< 0.5maxrotation/minStepsToFullRoation is the maximum of one change
  }),
});

class StandBodyRotationProvider : public StandBodyRotationProviderBase
{
public:
  StandBodyRotationProvider() { i = leftSum = rightSum = 0; changeValue = 0; }
  void update(StandBodyRotation& standBodyRotation);

private:
  unsigned i;
  unsigned leftSum;
  unsigned rightSum;
  float changeValue;
};
