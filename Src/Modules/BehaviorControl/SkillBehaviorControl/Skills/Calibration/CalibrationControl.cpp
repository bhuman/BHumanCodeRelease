/**
 * @file CalibrationControl.cpp
 *
 * This file defines skill that defines the behavior of the robot to start the different calibration steps.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) CalibrationControl,
       vars((bool)(false) cameraCalibrationFinished,
            (bool)(false) initialized,
            (CalibrationRequest) theCalibrationRequest))
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
        if(!cameraCalibrationFinished)
          goto decideForNextState;
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
        theCalibrationRequest.preciseJointPositions = true;
        CalibrateRobot({.request = theCalibrationRequest });
      }
      Stand();
      LookLeftAndRight({.startLeft = false});
    }
  }

  state(decideForNextState)
  {
    transition
    {
      if(!cameraCalibrationFinished)
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
      ManualCameraCalibration();
    }
  }

  state(calibrationFinished)
  {
    CalibrationFinished();
    Say({.text = "Calibration finished."});
    Stand();
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
