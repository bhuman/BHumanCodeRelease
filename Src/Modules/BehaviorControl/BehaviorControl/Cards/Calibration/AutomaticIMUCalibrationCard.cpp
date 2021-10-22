/**
 * @file AutomaticIMUCalibrationCard.cpp
 *
 * @author Philip Reichenberg
 */

#include "Platform/File.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(AutomaticIMUCalibrationCard,
{,
  CALLS(Activity),
  CALLS(CalibrateRobot),
  CALLS(LookAtAngles),
  CALLS(Say),
  CALLS(Stand),
  USES(CalibrationRequest),
  REQUIRES(FrameInfo),
  USES(IMUCalibration),
  DEFINES_PARAMETERS(
  {,
    (int)(1500) waitTime,
  }),
});

class AutomaticIMUCalibrationCard : public AutomaticIMUCalibrationCardBase
{
  bool preconditions() const override
  {
    return !isFinished && !theIMUCalibration.isCalibrated;
  }

  bool postconditions() const override
  {
    return isFinished;
  }

  void reset() override
  {
    startTimestamp = theFrameInfo.time;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::unknown);
    theStandSkill();
    theLookAtAnglesSkill(0_deg, 0_deg);
    if(theFrameInfo.getTimeSince(startTimestamp) > waitTime)
    {
      theSaySkill("calibrating i m u");
      CalibrationRequest calibrationRequest = theCalibrationRequest;
      calibrationRequest.serialNumberIMUCalibration++;
      theCalibrateRobotSkill(calibrationRequest);
      isFinished = true;
    }
  }

  unsigned startTimestamp = 0;
  bool isFinished = false;
};

MAKE_CARD(AutomaticIMUCalibrationCard);
