/**
 * @file AutomaticIMUCalibration.cpp
 * This skill lets the robot stand still for a moment and calibrates the IMU.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) AutomaticIMUCalibration)
{
  initial_state(initial)
  {
    transition
    {
      if(state_time)
        goto waiting;
    }
    action
    {
      Stand();
      LookAtAngles({.pan = 0_deg,
                    .tilt = 0_deg});
      Say({.text = "calibrating i m u"});
      CalibrationRequest calibrationRequest = theCalibrationRequest;
      calibrationRequest.preciseJointPositions = true;
      calibrationRequest.serialNumberIMUCalibration = theIMUCalibration.serialNumberIMUCalibration + 1;
      CalibrateRobot({.request = calibrationRequest});
    }
  }

  state(waiting)
  {
    transition
    {
      if(theIMUCalibration.isCalibrated &&
         theIMUCalibration.serialNumberIMUCalibration == theCalibrationRequest.serialNumberIMUCalibration)
        goto done;
    }
    action
    {
      Stand();
      LookAtAngles({.pan = 0_deg,
                    .tilt = 0_deg});
    }
  }

  target_state(done)
  {
    action
    {
      Stand();
      LookAtAngles({.pan = 0_deg,
                    .tilt = 0_deg});

      CalibrateRobot({.request = CalibrationRequest()});
    }
  }
}
