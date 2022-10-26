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

MAKE_MODULE(SkillBehaviorControl, behaviorControl, SkillBehaviorControl::getExtModuleInfo);

SkillBehaviorControl::SkillBehaviorControl() :
  Cabsl<SkillBehaviorControl>(const_cast<ActivationGraph*>(&theActivationGraph)),
  theSkillRegistry("skills.cfg", const_cast<ActivationGraph&>(theActivationGraph), theArmMotionRequest, theBehaviorStatus, theCalibrationRequest, theHeadMotionRequest, theMotionRequest)
{}

std::vector<ModuleBase::Info> SkillBehaviorControl::getExtModuleInfo()
{
  auto result = SkillBehaviorControl::getModuleInfo();
  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
  return result;
}

void SkillBehaviorControl::update(ActivationGraph&)
{
  theBehaviorStatus.activity = BehaviorStatus::unknown;
  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.passOrigin = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.speed = 0.f;
  theBehaviorStatus.shootingTo = Vector2f::Zero();

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  theHeadMotionRequest.pan = JointAngles::off;
  theHeadMotionRequest.tilt = JointAngles::off;
  theHeadMotionRequest.speed = 150_deg;
  theHeadMotionRequest.stopAndGoMode = false;

  theMotionRequest.odometryData = theOdometryData;
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballEstimateTimestamp = theFrameInfo.time;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;

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
    SystemCall::say((std::string(TypeRegistry::getEnumName(theGameState.ownTeam.color)) + " " + std::to_string(theGameState.playerNumber) + " " +
                     std::regex_replace(TypeRegistry::getEnumName(theRobotHealth.jointWithMaxTemperature), std::regex("([A-Z])"), " $1") + " " +
                     std::to_string(theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature])).c_str(), true);

  theSkillRegistry.modifyAllParameters();

  theSkillRegistry.preProcess(theFrameInfo.time);
  beginFrame(theFrameInfo.time);

  select_option(options);

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
        const Pose2f targetPose = theRobotPose.inversePose * theSkillRequest.target;
        if(theFieldBall.ballWasSeen(7000) || theTeammatesBallModel.isValid)
        {
          theWalkToPointObstacleSkill({ .target = theRobotPose.inversePose * theSkillRequest.target,
                                        .rough = theGameState.isGoalkeeper(),
                                        .disableObstacleAvoidance = theGameState.isGoalkeeper(),
                                        .targetOfInterest = theFieldBall.recentBallPositionRelative() }); // TODO: set the right parameters and occasionally use WalkPotentialField
          theLookAtBallAndTargetSkill({ .startTarget = true,
                                        .walkingDirection = targetPose.translation,
                                        .ballPositionAngle = theFieldBall.recentBallPositionRelative().angle() });
        }
        else
        {
          theWalkToPointObstacleSkill({ .target = theRobotPose.inversePose * theSkillRequest.target,
                                        .rough = theGameState.isGoalkeeper(),
                                        .disableObstacleAvoidance = theGameState.isGoalkeeper() }); // TODO: set the right parameters and occasionally use WalkPotentialField
          theLookActiveSkill({ .withBall = true });
        }
        break;
      }
      case SkillRequest::block:
        theBlockSkill({.target = theRobotPose.inversePose * theSkillRequest.target.translation});
        break;
      case SkillRequest::mark:
        theMarkSkill({.target = theRobotPose.inversePose * theSkillRequest.target.translation});
        break;
      case SkillRequest::observe:
        theObservePointSkill({.target = theRobotPose.inversePose * theSkillRequest.target.translation});
        break;
      default:
        FAIL("Unknown skill request.");
    }
  }
}
