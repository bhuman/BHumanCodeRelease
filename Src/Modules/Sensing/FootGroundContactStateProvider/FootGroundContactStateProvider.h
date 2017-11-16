/**
 * @file Representations/Sensing/FootGroundContactStateProvider.h
 * @author Alexis Tsogias
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FootGroundContactState.h"
#include "Tools/Module/Module.h"
#include "Tools/Range.h"
#include "Tools/RingBufferWithSum.h"

MODULE(FootGroundContactStateProvider,
{,
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FsrSensorData),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  PROVIDES(FootGroundContactState),
  DEFINES_PARAMETERS(
  {,
    (float)(0.2f) initialMinPressure,
    (float)(0.02f) noContactMargin,
    (float)(0.01f) maxLossRate, /**< per second*/
  }),
});

class FootGroundContactStateProvider : public FootGroundContactStateProviderBase
{
public:
  FootGroundContactStateProvider();

private:
  // an offset to calibrate the zero position.
  std::array<float, FsrSensors::numOfFsrSensors> leftZeroOffset;
  std::array<float, FsrSensors::numOfFsrSensors> rightZeroOffset;

  // calibration of the upper bounds
  std::array<float, FsrSensors::numOfFsrSensors> leftMax;
  std::array<float, FsrSensors::numOfFsrSensors> rightMax;

  std::array<float, FsrSensors::numOfFsrSensors> leftCalibrated; /**< Values of the four pressure sensors in the left foot (in kg) */
  std::array<float, FsrSensors::numOfFsrSensors> rightCalibrated; /**< Values of the four pressure sensors in the right foot (in kg) */

  void update(FootGroundContactState& footGroundContactState);
};
