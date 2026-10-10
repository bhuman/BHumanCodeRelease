#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/InertialData.h"
#include "Framework/Module.h"

MODULE(InertialDataProvider,
{,
  REQUIRES(CalibrationRequest),
  USES(MotionInfo),
  REQUIRES(InertialSensorData),
  PROVIDES(InertialData),
});

class InertialDataProvider : public InertialDataProviderBase
{
private:
  void update(InertialData& inertialData) override;
};
