/**
 * @file AutomaticIMUCalibration.cpp
 * This skill lets the robot stand still for a moment and calibrates the IMU.
 *
 * @author Philip Reichenberg
 */

#include "Platform/File.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"

SKILL_IMPLEMENTATION(AutomaticIMUCalibrationImpl,
{,
  IMPLEMENTS(AutomaticIMUCalibration),
  CALLS(CalibrateRobot),
  CALLS(LookAtAngles),
  CALLS(Say),
  CALLS(Stand),
  USES(CalibrationRequest),
  REQUIRES(FrameInfo),
  USES(IMUCalibration),
});

class AutomaticIMUCalibrationImpl : public AutomaticIMUCalibrationImplBase
{
  void reset(const AutomaticIMUCalibration&) override
  {
    startTimestamp = theFrameInfo.time;
  }

  void execute(const AutomaticIMUCalibration&) override
  {
    theStandSkill();
    theLookAtAnglesSkill({.pan = 0_deg,
                          .tilt = 0_deg});
    if(theFrameInfo.getTimeSince(startTimestamp) == 0)
    {
      theSaySkill({.text = "calibrating i m u"});
      CalibrationRequest calibrationRequest = theCalibrationRequest;
      calibrationRequest.serialNumberIMUCalibration = theIMUCalibration.serialNumberIMUCalibration + 1;
      theCalibrateRobotSkill({.request = calibrationRequest});
    }
  }

  bool isDone(const AutomaticIMUCalibration&) const override
  {
    return theIMUCalibration.isCalibrated &&
           theIMUCalibration.serialNumberIMUCalibration == theCalibrationRequest.serialNumberIMUCalibration;
  }

  unsigned startTimestamp = 0;
};

MAKE_SKILL_IMPLEMENTATION(AutomaticIMUCalibrationImpl);
