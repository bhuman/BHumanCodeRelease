#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Module/Module.h"

MODULE(JointAnglesProvider,
{,
  REQUIRES(RobotInfo),
  REQUIRES(JointSensorData),
  PROVIDES(JointAngles),
});

class JointAnglesProvider : public JointAnglesProviderBase
{
  void update(JointAngles& jointAngles) override;
};
