/**
 * @file SkillBehaviorControl.cpp
 *
 * This file implements the behavior control for the skill layer.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Streaming/TypeRegistry.h"
#include <string>

MAKE_MODULE(SkillBehaviorControl, SkillBehaviorControl::getExtModuleInfo);

SkillBehaviorControl::SkillBehaviorControl() :
  Cabsl<SkillBehaviorControl>(const_cast<ActivationGraph*>(&theActivationGraph)),
  theSkillRegistry("skills.cfg", const_cast<ActivationGraph&>(theActivationGraph), theArmMotionRequest, theBehaviorStatus, theCalibrationRequest, theHeadMotionRequest, theMotionRequest, theOptionalImageRequest)
{}

std::vector<ModuleBase::Info> SkillBehaviorControl::getExtModuleInfo()
{
  auto result = SkillBehaviorControl::getModuleInfo();
  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
  return result;
}

void SkillBehaviorControl::update(ActivationGraph&)
{
  DECLARE_DEBUG_RESPONSE("option:HandleRefereeSignal:now");

  theBehaviorStatus.calibrationFinished = false;
  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.passOrigin = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.speed = 0.f;
  theBehaviorStatus.shootingTo.reset();

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  theHeadMotionRequest.pan = JointAngles::off;
  theHeadMotionRequest.tilt = JointAngles::off;
  theHeadMotionRequest.speed = 150_deg;
  theHeadMotionRequest.stopAndGoMode = false;

  theMotionRequest.shouldInterceptBall = false;
  theMotionRequest.odometryData = theOdometryData;
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballEstimateTimestamp = theFrameInfo.time;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
  theMotionRequest.shouldInterceptBall = useNewHandleCatchBallBehavior && theFieldBall.interceptBall;

  theOptionalImageRequest.sendImage = false;

  // On request, tell the user whether a USB drive is mounted.
  if(theEnhancedKeyStates.hitStreak[KeyStates::headMiddle] == 3)
  {
    if(SystemCall::usbIsMounted())
      SystemCall::say("USB mounted");
    else
      SystemCall::say("USB not mounted");
  }

  // On request, tell the user the temperature of the hottest joint.
  if((theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 3 && theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 0) ||
     (theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3 && theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > 0))
    SystemCall::say((std::string(TypeRegistry::getEnumName(theGameState.color())) + " " + std::to_string(theGameState.playerNumber) + " " +
                     std::regex_replace(TypeRegistry::getEnumName(theRobotHealth.jointWithMaxTemperature), std::regex("([A-Z])"), " $1") + " " +
                     std::to_string(theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature])).c_str(), true);
  else if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3)
  {
    // On request, tell the user the deployed configuration of this robot.
    std::string wavName = Global::getSettings().headName.c_str();
    wavName.append(".wav");
    SystemCall::playSound(wavName.c_str());
    SystemCall::say(("is wearing a " +
                     std::string(TypeRegistry::getEnumName(theGameState.color())) +
                     " jersey with the number " +
                     std::to_string(theGameState.playerNumber) +
                     ". It is deployed in " +
                     std::string(TypeRegistry::getEnumName(theRobotHealth.configuration)) +
                     " for the " +
                     theRobotHealth.scenario +
                     " scenario at the " +
                     theRobotHealth.location +
                     " location.").c_str(), true, 0.9f);
  }

  theSkillRegistry.modifyAllParameters();

  theSkillRegistry.preProcess(theFrameInfo.time);
  beginFrame(theFrameInfo.time);

  PlaySoccer();

  endFrame();
  theSkillRegistry.postProcess();

  theLibCheck.performCheck();
}

void SkillBehaviorControl::executeRequest()
{
  if(theSkillRequest.skill == SkillRequest::none)
  {
    if(theFrameInfo.getTimeSince(timeWhenAnnouncedEmptySkillRequest) > 5000)
    {
      ANNOTATION("Behavior", "Skill request is empty");
      SystemCall::say("Skill request is empty");
      timeWhenAnnouncedEmptySkillRequest = theFrameInfo.time;
    }
  }
  else if(theSkillRequest.skill == SkillRequest::shoot ||
          theSkillRequest.skill == SkillRequest::pass ||
          theSkillRequest.skill == SkillRequest::dribble)
  {
    thePlayBallSkill();
  }
  else
  {
    for(const Teammate& teammate : theTeamData.teammates)
    {
      if(teammate.theBehaviorStatus.passTarget == theGameState.playerNumber)
      {
        theBehaviorStatus.passOrigin = teammate.number;
        theReceivePassSkill({.playerNumber = teammate.number});
        return;
      }
    }

    switch(theSkillRequest.skill)
    {
      case SkillRequest::stand:
        theStandSkill();
        theLookActiveSkill({.withBall = true});
        break;
      case SkillRequest::walk:
      {
        const Pose2f targetPose = theRobotPose.inverse() * theSkillRequest.target;
        if((theFieldBall.ballWasSeen(7000) || theTeammatesBallModel.isValid) && theFieldBall.isBallPositionConsistentWithGameState())
        {
          theWalkToPointObstacleSkill({.target = theRobotPose.inverse() * theSkillRequest.target,
                                       .rough = theGameState.isGoalkeeper(),
                                       .disableObstacleAvoidance = theGameState.isGoalkeeper(),
                                       .targetOfInterest = theFieldBall.recentBallPositionRelative()}); // TODO: set the right parameters and occasionally use WalkPotentialField
          if(theMotionInfo.isMotion(MotionPhase::stand))
            theLookActiveSkill({.withBall = true});
          else
            theLookAtBallAndTargetSkill({.startTarget = false,
                                         .walkingDirection = targetPose.translation,
                                         .ballPositionAngle = theFieldBall.recentBallPositionRelative().angle()});
        }
        else
        {
          theWalkToPointObstacleSkill({.target = theRobotPose.inverse() * theSkillRequest.target,
                                       .rough = theGameState.isGoalkeeper(),
                                       .disableObstacleAvoidance = theGameState.isGoalkeeper()}); // TODO: set the right parameters and occasionally use WalkPotentialField
          theLookActiveSkill({.withBall = true});
        }
        break;
      }
      case SkillRequest::block:
        theBlockSkill({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      case SkillRequest::mark:
        theMarkSkill({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      case SkillRequest::observe:
        theObservePointSkill({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      default:
        FAIL("Unknown skill request.");
    }
  }
}
