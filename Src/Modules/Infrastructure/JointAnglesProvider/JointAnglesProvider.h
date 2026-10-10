#pragma once

#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Framework/Module.h"

MODULE(JointAnglesProvider,
{,
  REQUIRES(JointCalibration),
  REQUIRES(JointSensorData),
  PROVIDES(JointAngles),
});

class JointAnglesProvider : public JointAnglesProviderBase
{
  void update(JointAngles& jointAngles) override;
};
