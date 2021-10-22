/**
 * @file CalibrateRobot.cpp
 *
 *
 * @author Lukas Plecher
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CalibrationRequest.h"

SKILL_IMPLEMENTATION(CalibrateRobotImpl,
{,
  IMPLEMENTS(CalibrateRobot),
  MODIFIES(CalibrationRequest),
});

class CalibrateRobotImpl : public CalibrateRobotImplBase
{
  void execute(const CalibrateRobot& p) override
  {
    theCalibrationRequest = p.request;
  }
};

MAKE_SKILL_IMPLEMENTATION(CalibrateRobotImpl);
