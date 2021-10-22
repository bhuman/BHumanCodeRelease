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
#include "Representations/Sensing/InertialData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

MODULE(IMUCalibrationProvider,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(InertialData),
  REQUIRES(InertialSensorData),
  USES(IMUCalibration),
  PROVIDES(IMUCalibration),
});

class IMUCalibrationProvider : public IMUCalibrationProviderBase
{
private:
  unsigned serialNumberIMUCalibration = 0;
  void update(IMUCalibration& imuCalibration) override;

public:
  IMUCalibrationProvider();
};
