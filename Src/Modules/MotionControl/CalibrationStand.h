#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Module.h"

MODULE(CalibrationStand,
{,
  REQUIRES(RobotDimensions),
  PROVIDES(StandArmRequest),
  PROVIDES(StandLegRequest),
  DEFINES_PARAMETERS(
  {,
    (float)(230) standHeight,
    (float)(50) footOffsetY,
    (int)(70) legStiffness,
  }),
});

class CalibrationStand : public CalibrationStandBase
{
private:
  void update(StandArmRequest& standArmRequest);
  void update(StandLegRequest& standLegRequest);

  void drawHelplines();
};
