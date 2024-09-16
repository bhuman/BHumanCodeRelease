/**
 * @file IMUCalibrationProvider.h
 *
 * This file declares a module that calibrates the IMUCalibration.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Sensing/InertialData.h"

#include "Debugging/DebugDrawings.h"
#include "Framework/Module.h"

MODULE(IMUCalibrationProvider,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(IMUValueState),
  REQUIRES(InertialData),
  REQUIRES(RawInertialSensorData),
  USES(IMUCalibration),
  PROVIDES(IMUCalibration),
  LOADS_PARAMETERS(
  {,
    (int) minStandStillTime,
    (int) waitTimeTillNewCalibrationAccepted,
    (int) minTimeBetweenCalibration,
    (bool) isAutoCalibrationActive,
  }),
});

class IMUCalibrationProvider : public IMUCalibrationProviderBase
{
private:
  void update(IMUCalibration& imuCalibration) override;

  bool isStandingStill();

  IMUCalibration tempIMUCalibration;
  unsigned int calibrationStarted = 0;
  unsigned int lastCalibration = 0;

public:
  IMUCalibrationProvider();
};
