/**
 * @file AutomaticFootSoleCalibration.cpp
 * This skill lets the robot walk to a position of the field without a field line.
 * Afterwards the calibration starts for the foot sole rotations.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Math/Rotation.h"
#include "Platform/File.h"
#include <filesystem>
// CABSL must be included last:
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(AutomaticFootSoleRotationCalibrationImpl,
{,
  IMPLEMENTS(AutomaticFootSoleRotationCalibration),
  CALLS(Activity),
  CALLS(CalibrateFootSole),
  CALLS(CalibrateRobot),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(TurnAngle),
  CALLS(WalkToPoint),
  REQUIRES(FieldDimensions),
  REQUIRES(FootSoleRotationCalibration),
  REQUIRES(InertialData),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(300.f) penaltyAreaDistance,
    (float)(500.f) sideLineDistance,
    (float)(300.f) centerCircleDistance,
  }),
});

class AutomaticFootSoleRotationCalibrationImpl : public AutomaticFootSoleRotationCalibrationImplBase
{
  option(AutomaticFootSoleRotationCalibration)
  {
    theLookForwardSkill();

    initial_state(initial)
    {
      transition
      {
        if(state_time > 0)
          goto walkToPointOnField;
      }
      action
      {
        if(!checkBorder)
        {
          footSoleId = theFootSoleRotationCalibration.id;
          currentFootSoleId = theFootSoleRotationCalibration.id;
          bool isFieldToSmall = false;
          if(bottomLeftBorder.x() > topRightBorder.x())
          {
            const int middlePoint = (bottomLeftBorder.x() + topRightBorder.x()) / 2;
            bottomLeftBorder.x() = middlePoint;
            topRightBorder.x() = middlePoint;
            isFieldToSmall = true;
          }
          if(bottomLeftBorder.y() > topRightBorder.y())
          {
            const int middlePoint = (bottomLeftBorder.y() + topRightBorder.y()) / 2;
            bottomLeftBorder.y() = middlePoint;
            topRightBorder.y() = middlePoint;
            isFieldToSmall = true;
          }
          checkBorder = true;
          if(isFieldToSmall)
            theSaySkill({.text = "Your field is too small! But I will try anyway"});
        }
        goalPosition = theRobotPose.translation;
        Geometry::clipPointInsideRectangle(bottomLeftBorder, topRightBorder, goalPosition);
        theStandSkill();
      }
    }

    state(walkToPointOnField)
    {
      transition
      {
        if(theWalkToPointSkill.isDone())
        {
          goto checkNumberOfCalibrations;
        }
      }
      action
      {
        theWalkToPointSkill({.target = theRobotPose.inversePose * goalPosition,
                             .disableObstacleAvoidance = true});
      }
    }

    state(checkNumberOfCalibrations)
    {
      transition
      {
        CalibrationRequest calibrationRequest;
        numberOfCalibrations = 2;
        calibrationRequest.numOfFootSoleCalibration = numberOfCalibrations;
        theCalibrateRobotSkill({.request = calibrationRequest});
        goto calibrate;
      }
      action
      {
        theStandSkill();
      }
    }

    state(calibrate)
    {
      transition
      {
        if(currentFootSoleId != theFootSoleRotationCalibration.id)
        {
          currentFootSoleId = theFootSoleRotationCalibration.id;

          if(currentFootSoleId - footSoleId >= numberOfCalibrations)
          {
            theSaySkill({.text = "Success: Calibrated Foot Sole!"});
            saveCalibration(theFootSoleRotationCalibration);
            goto done;
          }
          else
          {
            goto turnToNextPose;
          }
        }
      }
      action
      {
        theCalibrateFootSoleSkill();
      }
    }

    state(turnToNextPose)
    {
      transition
      {
        if(theTurnAngleSkill.isDone())
          goto calibrate;
      }
      action
      {
        theTurnAngleSkill({.angle = 360_deg / numberOfCalibrations});
      }
    }

    target_state(done)
    {
      action
      {
        theStandSkill();
      }
    }
  }

  static void saveCalibration(const FootSoleRotationCalibration& calibration)
  {
    const std::string path = std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().bodyName + "/Body";
    std::filesystem::create_directories(path);
    OutMapFile stream(path + "/footSoleRotationCalibration.cfg", true);
    stream << calibration;
  }

private:
  Vector2i bottomLeftBorder = Vector2i(static_cast<int>(theFieldDimensions.xPosOwnPenaltyArea + penaltyAreaDistance), static_cast<int>(theFieldDimensions.yPosRightPenaltyArea + sideLineDistance));
  Vector2i topRightBorder = Vector2i(static_cast<int>(-theFieldDimensions.centerCircleRadius - centerCircleDistance), static_cast<int>(theFieldDimensions.yPosLeftPenaltyArea - sideLineDistance));
  bool checkBorder = false;
  Vector2f goalPosition;
  int numberOfCalibrations = 0;
  int footSoleId = 0;
  int currentFootSoleId = 0;
};

MAKE_SKILL_IMPLEMENTATION(AutomaticFootSoleRotationCalibrationImpl);
