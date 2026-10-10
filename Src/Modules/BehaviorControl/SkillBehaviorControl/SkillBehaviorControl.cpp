/**
 * @file SkillBehaviorControl.cpp
 *
 * This file implements the behavior control for the skill layer.
 *
 * @author Arne Hasselbring
 */

#define FULL_HEADER
#include "SkillBehaviorControl.h"
#include "Debugging/DebugDrawings3D.h"
#include "Platform/SystemCall.h"
#include "Streaming/TypeRegistry.h"
#include <string>

MAKE_MODULE(SkillBehaviorControl);

SkillBehaviorControl::SkillBehaviorControl() :
  Cabsl<SkillBehaviorControl>(const_cast<ActivationGraph*>(&theActivationGraph))
{}

void SkillBehaviorControl::update(ActivationGraph&)
{
  DECLARE_DEBUG_RESPONSE("option:ManualCameraCalibration:finishNow");
  DECLARE_DEBUG_RESPONSE("option:ManualCameraCalibration:prepareRecording");
  DECLARE_DEBUG_DRAWING("option:ClearBall:bonus", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:borders", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:clearBall", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:evaluation", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:zero", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DirectKickOff:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DribbleTargetProvider:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DribbleTargetProvider:direction", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DribbleTargetProvider:step", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:HandleBallAtOwnGoalPost:blueprintWheel", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("debug data:option:HandleBallAtOwnGoalPost:drawKickType");
  DECLARE_DEBUG_DRAWING("option:HandleBallAtOwnGoalPost:kickWheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:goalLineIntersection", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:cullLine", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:goalSector", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:primaryRange", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Mark:markPose", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:PassToTeammate:target", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:PassToTeammate:precision", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("option:PassToTeammate:trajectory", "field");
  DECLARE_DEBUG_DRAWING("option:ReceivePass:target", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:WalkPotentialField:angles", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:WalkPotentialField:potentialField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:WalkToPointObstacle:obstacleAtMyPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:kickDirections", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:kicks", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:wheelSteal", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:sidewardRange", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:sideSteal", "drawingOnField");

  theBehaviorStateInformation.transitionFailed = false;
  theBehaviorStateInformation.remoteControlActive = false;

  theBehaviorStatus.calibrationFinished = false;
  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.passOrigin = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.speed = 0.f;
  theBehaviorStatus.shootingTo.reset();

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  theHeadMotionRequest.pan = JointAngles::off;
  theHeadMotionRequest.tilt = JointAngles::off;
  theHeadMotionRequest.speed = 150_deg;
  theHeadMotionRequest.stopAndGoMode = false;

  theMotionRequest.odometryData = theOdometryData;
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballEstimateTimestamp = theFrameInfo.time;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
  theMotionRequest.targetOfInterest.reset();

  if(theEnhancedKeyStates.hitStreak[KeyStates::walk] == 3 && !theEnhancedKeyStates.pressed[KeyStates::stand])
  {
    // On request, tell the user the deployed configuration of this robot.
    SystemCall::say((Global::getSettings().robotName +
                     " is wearing a " +
                     std::string(TypeRegistry::getEnumName(theGameState.color())) +
                     " jersey with the number " +
                     std::to_string(theGameState.playerNumber) +
                     ". It is deployed in " +
                     std::string(TypeRegistry::getEnumName(theRobotHealth.configuration)) +
                     " for the scenario " +
                     theRobotHealth.scenario +
                     " at the location " +
                     theRobotHealth.location +
                     ".").c_str(), true, 0.9f);
  }

  beginFrame(theFrameInfo.time);
  execute(root);
  endFrame();

  theMotionRequest.shouldInterceptBall = theInterceptBallInformation.intercept && !theBehaviorStateInformation.remoteControlActive;
  theMotionRequest.shouldWalkOutOfBallLine = theInterceptBallInformation.walkOutOfBallPath && !theBehaviorStateInformation.remoteControlActive;

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
          theSkillRequest.skill == SkillRequest::dribble ||
          theSkillRequest.skill == SkillRequest::clear)
  {
    PlayBall();
  }
  else if(theSkillRequest.skill == SkillRequest::waitShoot)
    GoToBallWaitAndKick();
  else
  {
    // Check if a pass is communicated
    for(const Teammate& teammate : theTeamData.teammates)
    {
      if(teammate.theBehaviorStatus.passTarget == theGameState.playerNumber)
      {
        // Communicated message is not too old
        if(theFrameInfo.getTimeSince(teammate.theFrameInfo.time) < ignoreReceivePassAfterTime)
        {
          lastReceivePassRequestTimestamp = theFrameInfo.time;
          receivePassPlayerNumber = teammate.number;
          theBehaviorStatus.passOrigin = teammate.number;
          ReceivePass({.playerNumber = teammate.number});
          return;
        }
        else // Communicated message is too old
        {
          lastReceivePassRequestTimestamp = 0;
          receivePassPlayerNumber = -1;
        }
      }
    }
    // Continue ReceivePass after communication stopped. Assume the sender just kicked the ball
    if(theFrameInfo.getTimeSince(lastReceivePassRequestTimestamp) < continueReceivePassTime && receivePassPlayerNumber != -1)
    {
      theBehaviorStatus.passOrigin = receivePassPlayerNumber;
      ReceivePass({.playerNumber = receivePassPlayerNumber});
      return;
    }
    receivePassPlayerNumber = -1;

    // slow head movements of LookActive down if the ball is far away
    isSlowingDownLookActive = (theFieldBall.positionOnField - theRobotPose.translation).squaredNorm() > sqr(2500.f + (isSlowingDownLookActive ? 0.f : 300.f));

    switch(theSkillRequest.skill)
    {
      case SkillRequest::stand:
        Stand();
        LookActive({.withBall = true,
                    .slowdownFactor = isSlowingDownLookActive ? 0.4f : 1.f});
        break;
      case SkillRequest::walk:
      {
        const Pose2f targetPose = theRobotPose.inverse() * theSkillRequest.target;
        const ReduceWalkSpeedType::ReduceWalkSpeedType reduceWalkSpeedType = theGameState.isGoalkeeper() ? ReduceWalkSpeedType::distanceBased : ReduceWalkSpeedType::normal;

        if(theFieldBall.ballWasSeen(7000) || theTeamBallModel.isValid)
        {
          WalkToPointObstacle({.target = theRobotPose.inverse() * theSkillRequest.target,
                               .reduceWalkSpeedType = reduceWalkSpeedType,
                               .disableObstacleAvoidance = theGameState.isGoalkeeper() && (!theGameState.isGoalKick() || !theGameState.isForOwnTeam()),
                               .targetOfInterest = theFieldBall.recentBallPositionRelative()}); // TODO: set the right parameters

          if(theMotionInfo.isMotion(MotionPhase::stand))
            LookActive({.withBall = true,
                        .slowdownFactor = isSlowingDownLookActive ? 0.4f : 1.f});
          else
            LookAtBallAndTarget({.startBall = true,
                                 .walkingDirection = targetPose.translation});
        }
        else
        {
          WalkToPointObstacle({.target = theRobotPose.inverse() * theSkillRequest.target,
                               .reduceWalkSpeedType = reduceWalkSpeedType,
                               .disableObstacleAvoidance = theGameState.isGoalkeeper()}); // TODO: set the right parameters
          LookActive();
        }
        break;
      }
      case SkillRequest::block:
        Block({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      case SkillRequest::mark:
        Mark({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      case SkillRequest::observe:
        ObservePoint({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      default:
        FAIL("Unknown skill request.");
    }
  }
}

void SkillBehaviorControl::handleWalkSpeedAtFieldBorder()
{
  // TODO Special Handling for KickIns/CornerKicks
  Vector2f robotPose = theRobotPose.translation;
  const float clipRatio = Rangef::ZeroOneRange().limit(theFieldDimensions.clipToField(robotPose, fieldBorderOffset) / outSideFieldBorderRange);
  Pose2f maxWalkSpeed(std::min(std::abs(maxWalkSpeedAtFieldBorder.rotation / theWalkingEngineOutput.maxSpeed.rotation), std::abs(theMotionRequest.walkSpeed.rotation)) * (std::signbit(theMotionRequest.walkSpeed.rotation) ? -1.f : 1.f),
                      std::min(std::abs(maxWalkSpeedAtFieldBorder.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x()), std::abs(theMotionRequest.walkSpeed.translation.x())) * (std::signbit(theMotionRequest.walkSpeed.translation.x()) ? -1.f : 1.f),
                      std::min(std::abs(maxWalkSpeedAtFieldBorder.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y()), std::abs(theMotionRequest.walkSpeed.translation.y())) * (std::signbit(theMotionRequest.walkSpeed.translation.y()) ? -1.f : 1.f));

  theMotionRequest.walkSpeed.translation = clipRatio * maxWalkSpeed.translation + (1.f - clipRatio) * theMotionRequest.walkSpeed.translation;
  theMotionRequest.walkSpeed.rotation = clipRatio * maxWalkSpeed.rotation + (1.f - clipRatio) * theMotionRequest.walkSpeed.rotation;
}
