#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Module/Module.h"

MODULE(JointAnglesProvider,
{,
  REQUIRES(JointSensorData),
  PROVIDES(JointAngles),
});

class JointAnglesProvider : public JointAnglesProviderBase
{
  void update(JointAngles& jointAngles) override;
};
