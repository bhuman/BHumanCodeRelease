#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Tools/Module/Module.h"

MODULE(CalibrationStand,
{,
  REQUIRES(RobotDimensions),
  REQUIRES(HeadJointRequest),
  PROVIDES(StandOutput),
  DEFINES_PARAMETERS(
  {,
    (float)(220) standHeight,
    (float)(50) footOffsetY,
    (bool)(true) useRequestedHeadAngle,
  }),
});

class CalibrationStand : public CalibrationStandBase
{
private:
  void update(StandOutput& standOutput);

  void drawHelplines();
};
