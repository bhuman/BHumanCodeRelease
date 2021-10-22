/**
 * @file AutomaticFootSoleCalibrationCard.cpp
 *
 * @author Philip Reichenberg
 */

#include "Platform/File.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(AutomaticFootSoleCalibrationCard,
{,
  CALLS(Activity),
  CALLS(CalibrateRobot),
  CALLS(KeyFrameSingleArm),
  CALLS(LookAtAngles),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkToPoint),
  USES(CalibrationRequest),
  USES(FootSoleRotationCalibration),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (int)(3) maxMoveForwardTries,
    (int)(6) maxMoveTries,
  }),
});

class AutomaticFootSoleCalibrationCard : public AutomaticFootSoleCalibrationCardBase
{
  bool preconditions() const override
  {
    return !isFinished && !theFootSoleRotationCalibration.isCalibrated;
  }

  bool postconditions() const override
  {
    return isFinished;
  }

  option
  {
    initial_state(start)
    {
      transition
      {
        if(state_time > 0)
          goto initialTurn;
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0_deg, 0_deg);
        theStandSkill();
        theSaySkill("calibrating feet sole rotation");
      }
    }

    state(initialTurn)
    {
      transition
      {
        if(std::abs(theRobotPose.rotation) < 20_deg)
          goto moveForward;
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0_deg, 0_deg);
        theWalkToPointSkill(Pose2f(45_deg, 0.f, 0.f), 0.7f, true, true, false, true);
      }
    }

    state(standing)
    {
      transition
      {
        if(state_time > 10000)
          goto calibrate;
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0_deg, 0_deg);
        theStandSkill();
        CalibrationRequest calibrationRequest = theCalibrationRequest;
        calibrationRequest.preciseJointPositions = true;
        theCalibrateRobotSkill(calibrationRequest);
      }
    }

    state(calibrate)
    {
      transition
      {
        if(state_time > 100)
        {
          if(moveCounter > maxMoveTries)
            goto done;
          else
          {
            requestedCalibration = false;
            goto moveForward;
          }
        }
      }
      action
      {
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0_deg, 0_deg);
        theStandSkill();
        if(!requestedCalibration)
        {
          CalibrationRequest calibrationRequest = theCalibrationRequest;
          calibrationRequest.serialNumberFootSoleRotationCalibration++;
          calibrationRequest.preciseJointPositions = true;
          theCalibrateRobotSkill(calibrationRequest);
          requestedCalibration = true;
        }
      }
    }

    state(moveForward)
    {
      transition
      {
        if(moveCounter > maxMoveForwardTries)
          goto turn;
        else if(state_time > 4000)
          goto standing;
      }
      action
      {
        if(state_time == 0)
          moveCounter++;
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0_deg, 0_deg);
        if(state_time < 2000)
          theWalkToPointSkill(Pose2f(0_deg, 400.f, 0.f), 0.7f, true, true, false, true);
        else
          theWalkToPointSkill(Pose2f(0_deg, 0.f, 0.f), 0.7f, true, true, false, true);
      }
    }

    state(turn)
    {
      transition
      {
        if(state_time > 4000)
          goto standing;
      }
      action
      {
        if(state_time == 0)
          moveCounter++;
        theActivitySkill(BehaviorStatus::unknown);
        theLookAtAnglesSkill(0_deg, 0_deg);
        if(state_time < 2000)
          theWalkToPointSkill(Pose2f(90_deg, 0.f, 0.f), 0.7f, true, true, false, true);
        else
          theWalkToPointSkill(Pose2f(0_deg, 0.f, 0.f), 0.7f, true, true, false, true);
      }
    }

    state(done)
    {
      transition
      {
      }
      action
      {
        theLookAtAnglesSkill(0_deg, 0_deg);
        theActivitySkill(BehaviorStatus::unknown);
        isFinished = true;
        if(theFootSoleRotationCalibration.isCalibrated)
          theSaySkill("Foot sole calibration successful");
        CalibrationRequest calibrationRequest = theCalibrationRequest;
        calibrationRequest.preciseJointPositions = false;
        theCalibrateRobotSkill(calibrationRequest);
        theStandSkill();
      }
    }
  }

  bool requestedCalibration = false;
  bool isFinished = false;
  int moveCounter = 0;
};

MAKE_CARD(AutomaticFootSoleCalibrationCard);
