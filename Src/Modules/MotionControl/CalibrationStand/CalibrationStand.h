#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionUtilities.h"

MODULE(CalibrationStand,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(RobotDimensions),
  PROVIDES(StandArmRequest),
  PROVIDES(StandLegRequest),
  DEFINES_PARAMETERS(
  {,
    (float)(235) standHeight,
    (float)(50) footOffsetY,
    (int)(70) legStiffness,
    (int)(30) armStiffness,
    (unsigned int)(0) timestampLeg,
    (unsigned int)(0) timestampArm,
    (float)(2000) interpolationTimeLeg,
    (float)(2000) interpolationTimeArm,
    (bool)(true) needToInterpolateLeg,
    (bool)(true) needToInterpolateArm,
  }),
});

class CalibrationStand : public CalibrationStandBase
{
private:
  StandArmRequest startJointsLeg;
  StandArmRequest startJointsArm;
  void update(StandArmRequest& standArmRequest) override;
  void update(StandLegRequest& standLegRequest) override;
};
