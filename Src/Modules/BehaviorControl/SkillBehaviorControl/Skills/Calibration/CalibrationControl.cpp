/**
 * @file CalibrationControl.cpp
 *
 * This file defines skill that defines the behavior of the robot to start the different calibration steps.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GyroState.h"

SKILL_IMPLEMENTATION(CalibrationControlImpl,
{,
  IMPLEMENTS(CalibrationControl),
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(PlayDead),
  CALLS(Say),
  CALLS(Stand),
  CALLS(LookLeftAndRight),
  CALLS(AutomaticIMUCalibration),
  CALLS(AutonomousCameraCalibration),
  CALLS(AutomaticFootSoleRotationCalibration),
  REQUIRES(FallDownState),
  REQUIRES(FootSoleRotationCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(GyroState),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) lookAroundDuration, /**< Min duration to look around before starting the calibration process. */
    (int)(6000) lookAroundDurationMax, /**< Max duration to look around before starting the calibration process. */
  }),
});

class CalibrationControlImpl : public CalibrationControlImplBase
{
  option(CalibrationControl)
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
        if(!imuCalibrationFinished ||
           !theFootSoleRotationCalibration.footCalibration[Legs::left].isCalibrated || // TODO ignore these flags but execute the calibration always
           !theFootSoleRotationCalibration.footCalibration[Legs::right].isCalibrated ||
           !cameraCalibrationFinished)
          goto lookAroundBeforeCalibration;
        else
          goto calibrationFinished;
      }
      action
      {
        theStandSkill();
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
        theLookLeftAndRightSkill({.startLeft = false});
        theStandSkill();
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
        else if((!theFootSoleRotationCalibration.footCalibration[Legs::left].isCalibrated ||
                 !theFootSoleRotationCalibration.footCalibration[Legs::right].isCalibrated) &&
                !theFootSoleRotationCalibration.notCalibratable)
          goto calibrateFootSoleRotation;
        else
          goto calibrationFinished;
      }
      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(calibrateIMU)
    {
      transition
      {
        if(theAutomaticIMUCalibrationSkill.isDone())
        {
          imuCalibrationFinished = true;
          goto decideForNextState;
        }
      }
      action
      {
        theAutomaticIMUCalibrationSkill();
        if(theFrameInfo.getTimeSince(theGyroState.notMovingSinceTimestamp) < 50) // the time should be less than the robot needs to say the text
          theSaySkill("Please do not move me. I need to calibrate my i m u");
      }
    }

    state(calibrateCamera)
    {
      transition
      {
        if(theAutonomousCameraCalibrationSkill.isDone())
        {
          cameraCalibrationFinished = true;
          goto decideForNextState;
        }
      }
      action
      {
        theAutonomousCameraCalibrationSkill();
      }
    }

    state(calibrateFootSoleRotation)
    {
      transition
      {
        if(theAutomaticFootSoleRotationCalibrationSkill.isDone())
          goto decideForNextState;
      }
      action
      {
        theAutomaticFootSoleRotationCalibrationSkill();
      }
    }

    state(calibrationFinished)
    {
      theActivitySkill({.activity = BehaviorStatus::calibrationFinished});
      theSaySkill({.text = "Calibration finished."});
      thePlayDeadSkill();
      theLookForwardSkill();
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
        theStandSkill();
        theLookForwardSkill();
      }
    }
  }

  bool cameraCalibrationFinished = false;
  bool imuCalibrationFinished = false;
};

MAKE_SKILL_IMPLEMENTATION(CalibrationControlImpl);
