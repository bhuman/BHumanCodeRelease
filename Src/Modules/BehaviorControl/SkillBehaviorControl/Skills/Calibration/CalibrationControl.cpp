/**
 * @file CalibrationControl.cpp
 *
 * This file defines skill that defines the behavior of the robot to start the different calibration steps.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) CalibrationControl,
       defs((int)(3000) lookAroundDuration, /**< Min duration to look around before starting the calibration process. */
            (int)(6000) lookAroundDurationMax, /**< Max duration to look around before starting the calibration process. */
            (int)(3) notMovingCycles), /**< Sample size from gyro state times this factor to check if we can calibrate. */
       vars((bool)(false) cameraCalibrationFinished,
            (bool)(false) imuCalibrationFinished,
            (bool)(false) initialized,
            (CalibrationRequest)({}) theCalibrationRequest))
{
  common_transition
  {
    if(theFallDownState.state == FallDownState::fallen || theFallDownState.state == FallDownState::falling)
      goto waitAfterFallen;
  }

  initial_state(start)
  {
    transition
    {
      if(initialized)
      {
        if(!imuCalibrationFinished ||
           !cameraCalibrationFinished)
          goto lookAroundBeforeCalibration;
        else
          goto calibrationFinished;
      }
    }
    action
    {
      if(!initialized)
      {
        initialized = true;
        theCalibrationRequest = {};
        CalibrateRobot({.request = theCalibrationRequest });
      }
      Stand();
      LookLeftAndRight({.startLeft = false});
    }
  }

  state(lookAroundBeforeCalibration)
  {
    transition
    {
      if((state_time > lookAroundDuration && theRobotPose.quality != RobotPose::LocalizationQuality::poor) || state_time > lookAroundDurationMax)
        goto decideForNextState;
    }
    action
    {
      LookLeftAndRight({.startLeft = false});
      Stand();
    }
  }

  state(decideForNextState)
  {
    transition
    {
      if(!imuCalibrationFinished)
        goto calibrateIMU;
      else if(!cameraCalibrationFinished)
        goto calibrateCamera;
      else
        goto calibrationFinished;
    }
    action
    {
      LookForward();
      Stand();
    }
  }

  state(calibrateIMU)
  {
    transition
    {
      if(action_done)
      {
        imuCalibrationFinished = true;
        goto decideForNextState;
      }
    }
    action
    {
      if(theFrameInfo.getTimeSince(theIMUValueState.notMovingSinceTimestamp) < theIMUValueState.filterTimeWindow * notMovingCycles) // the time should be less than the robot needs to say the text
        Say({.text = "Please do not move me. I need to calibrate my i m u"});
      AutomaticIMUCalibration();
    }
  }

  state(calibrateCamera)
  {
    transition
    {
      if(action_done)
      {
        cameraCalibrationFinished = true;
        goto decideForNextState;
      }
    }
    action
    {
      AutonomousCameraCalibration();
    }
  }

  state(calibrationFinished)
  {
    CalibrationFinished();
    Say({.text = "Calibration finished."});
    PlayDead();
    LookForward();
  }

  state(waitAfterFallen)
  {
    transition
    {
      if(theFallDownState.state == FallDownState::upright)
        goto start;
    }
    action
    {
      Stand();
      LookForward();
    }
  }
}
