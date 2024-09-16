/**
 * @file KeyframePhaseBase.cpp
 * This file declares functions for the KeyframePhase.
 * Content: It contains all methods to handle the different engine states like selecting
 * a motion and switching from off to working to finished (and so on)
 * @author Philip Reichenberg
 */
#include "KeyframePhaseBase.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Modules/MotionControl/KeyframeMotionEngine/KeyframeMotionEngine.h"

KeyframePhaseBase::KeyframePhaseBase(KeyframeMotionEngine& engine, const KeyframeMotionRequest& keyframeMotionRequest) :
  MotionPhase(keyframeMotionRequest.keyframeMotion < KeyframeMotionID::firstNonGetUpAction ? MotionPhase::getUp : MotionPhase::keyframeMotion),
  engine(engine),
  currentKeyframeMotionRequest(keyframeMotionRequest)
{
  jointCompensationReducer.fill(1.f);
  motionListIDLoops.fill(0);
  blockIDLoops.fill(0);
}

bool KeyframePhaseBase::isInBreakUpRange()
{
  if(engine.stepKeyframes)
    return false;
  const Angle pitchVal = engine.theInertialData.angle.y();
  const Angle rollVal = engine.theInertialData.angle.x();

  if(((engine.theFilteredCurrent.legMotorMalfunction && engine.motorMalfunctionBreakUp) // motor malfunction
      || !currentTorsoAngleBreakUp.pitchDirection.isInside(pitchVal) // tilting forward/backward
      || !currentTorsoAngleBreakUp.rollDirection.isInside(rollVal)) // tilting left/right
     && breakUpOn)
  {
    safePreviousKeyframeData();
    if(SystemCall::getMode() != SystemCall::simulatedRobot)
      tryCounter += 1;
    state = EngineState::breakUp;
    breakUpTimestamp = engine.theFrameInfo.time; // TODO extra stuff for sitDown?
    currentMotion = KeyframeMotionList();
    currentMotionBlock.clear();
    currentKeyframe = Keyframe();
    waitForFallenCheck = true;
    if(SystemCall::getMode() != SystemCall::logFileReplay && !(engine.theFilteredCurrent.legMotorMalfunction && engine.motorMalfunctionBreakUp))
      SystemCall::say("Abort");
    for(std::size_t i = 0; i < jointRequestOutput.angles.size(); i++)
      jointRequestOutput.stiffnessData.stiffnesses[i] = 10;
    return true;
  }
  return false;
}

void KeyframePhaseBase::waitForFallen()
{
  if(waitForFallenCheck)
  {
    frontHackTriggered = false;
    breakUpTimestamp = engine.theFrameInfo.time; // TODO extra stuff for sitDown?
    waitForFallenCheck = false;
    // Set stiffness low and set head into a save position
    for(std::size_t i = 0; i < jointRequestOutput.angles.size(); i++)
      jointRequestOutput.stiffnessData.stiffnesses[i] = engine.safeFallParameters.bodyStiffness;
    // When falling backward
    if(engine.theInertialData.angle.y() < 0_deg)
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitBack], jointRequestOutput, static_cast<Joints::Joint>(0), Joints::numOfJoints);
    // When falling forward
    else
    {
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitFrontGetUp], jointRequestOutput, static_cast<Joints::Joint>(0), Joints::numOfJoints);
      jointRequestOutput.stiffnessData.stiffnesses[Joints::lHipPitch] = 30;
      jointRequestOutput.stiffnessData.stiffnesses[Joints::rHipPitch] = 30;
      jointRequestOutput.angles[Joints::lElbowRoll] = -40_deg;
      jointRequestOutput.angles[Joints::rElbowRoll] = 40_deg;
      jointRequestOutput.stiffnessData.stiffnesses[Joints::lElbowRoll] = 20;
      jointRequestOutput.stiffnessData.stiffnesses[Joints::rElbowRoll] = 20;
      jointRequestOutput.stiffnessData.stiffnesses[Joints::lElbowYaw] = 0;
      jointRequestOutput.stiffnessData.stiffnesses[Joints::rElbowYaw] = 0;
    }

    jointRequestOutput.stiffnessData.stiffnesses[Joints::headYaw] = engine.safeFallParameters.headStiffness;
    jointRequestOutput.stiffnessData.stiffnesses[Joints::headPitch] = engine.safeFallParameters.headStiffness;
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
    motionListIDLoops.fill(0);
    blockIDLoops.fill(0);
  }
  // Go back to motion selection in case robot is (or shall start) doing get up motions
  if(currentKeyframeMotionRequest.keyframeMotion < KeyframeMotionID::firstNonGetUpAction)
  {
    if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > 1000)
    {
      if(tryCounter > 1)
        frontHackTriggered = true;

      if(tryCounter >= engine.maxTryCounter)  // TODO
        state = EngineState::helpMeState;
      else
      {
        state = EngineState::decideAction;
        lastNotActiveTimestamp = engine.theFrameInfo.time;
      }
    }
  }
  // Break up happened in a non get up motion and the robot is upright again
  else if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > 1000 && std::abs(engine.theInertialData.angle.y()) < 30_deg && std::abs(engine.theInertialData.angle.x()) < 30_deg)
  {
    state = EngineState::decideAction;
    lastNotActiveTimestamp = engine.theFrameInfo.time;
    doSlowKeyframeAfterFall = true;
  }

  if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > engine.safeFallParameters.unstiffWaitTime)
  {
    FOREACH_ENUM(Joints::Joint, joint)
      jointRequestOutput.stiffnessData.stiffnesses[joint] = 0;
  }
  // We assume the head moved into a save position, so we can lower the stiffness
  else if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > engine.safeFallParameters.lowHeadStiffnessWaitTime)
  {
    jointRequestOutput.stiffnessData.stiffnesses[0] = engine.safeFallParameters.bodyStiffness;
    jointRequestOutput.stiffnessData.stiffnesses[1] = engine.safeFallParameters.bodyStiffness;
  }
  fastRecover = false;
  recoverLyingOnArmCheck = false;

  // Too many tries or a malfunction in a motor was detected
  if(tryCounter >= engine.maxTryCounter || (engine.theFilteredCurrent.legMotorMalfunction && engine.motorMalfunctionBreakUp))
    state = EngineState::helpMeState;
}

void KeyframePhaseBase::doHelpMeStuff()
{
  tooManyTries = true;
  wasHelpMe = true;
  if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > 100)
  {
    jointRequestOutput.stiffnessData.stiffnesses[0] = 0;
    jointRequestOutput.stiffnessData.stiffnesses[1] = 0;
  }
  if(std::abs(engine.theInertialData.angle.y()) < 30_deg && std::abs(engine.theInertialData.angle.x()) < 30_deg)
  {
    state = EngineState::finished;
    checkFinishedState();
    lastNotActiveTimestamp = engine.theFrameInfo.time;
  }
  if(engine.theFrameInfo.getTimeSince(helpMeTimestamp) > 5000)
  {
    SystemCall::say("Help me");
    SystemCall::playSound("mimimi.wav");
    helpMeTimestamp = engine.theFrameInfo.time;
  }
}

void KeyframePhaseBase::setNextJoints()
{
  // TODO just in case do some check?
  float useRatio = ratio;
  calculateJointDifference();
  removePreviousKeyframeJointCompensation();
  applyJointCompensation();

  targetJoints = lineJointRequest;
  ASSERT(currentKeyframe.interpolationType != InterpolationType::Default);

  if(currentKeyframe.interpolationType == SinusMinToMax)
    useRatio = 0.5f * std::sin(ratio * Constants::pi - Constants::pi / 2.f) + 0.5f;
  else if(currentKeyframe.interpolationType == SinusZeroToMax)
    useRatio = std::sin(ratio * Constants::pi / 2);
  MotionUtilities::interpolate(startJoints, targetJoints, useRatio, jointRequestOutput, engine.theJointAngles);
  FOREACH_ENUM(Joints::Joint, joint) // otherwise ignore gets overwritten and the motionCombinator can not set the ignored joints
    jointRequestOutput.angles[joint] = targetJoints.angles[joint] == JointAngles::ignore ? JointAngles::ignore : jointRequestOutput.angles[joint];

  lastUnbalanced = jointRequestOutput;

  if(balancerOn)
    pidWithCom();
  PLOT("module:KeyframeMotionEngine:ratio", useRatio);
}

void KeyframePhaseBase::checkFinishedState()
{
  // Robot could not get up and is now upright again
  if(wasHelpMe)
  {
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
    state = EngineState::working;
    safePreviousKeyframeData();
    // SET Stand motion
    tooManyTries = false;
    wasHelpMe = false;
    waitForFallenCheck = false;
    errorTriggered = false;
    currentMotion = engine.motions[KeyframeMotionListID::stand];
    // TODO Set stand motion
    initKeyframeMotion(true);
    breakUpOn = true;
    wasInWaiting = false;
    isCurrentLineOver();
    setNextJoints();
    return;
  }
  if(this->type == MotionPhase::getUp)
  {
    for(const auto& nextPhase : currentMotion.startAsNextPhaseMotion)
    {
      if(checkConditions(nextPhase.conditions, nextPhase.isAndCondition))
      {
        selectMotionToBeExecuted();
        return;
      }
    }
  }
  // The check for a torso angle in y-axis of less than 30deg is a fail save, in case something is wrong with the config settings
  // Motion finished and shall still balance to make sure robot will not fall when leaving the KeyframeMotionEngine
  if(currentMotion.balanceOut && std::abs(engine.theInertialData.angle.y()) < 30_deg)
    doBalanceOut();

  else if(currentKeyframeMotionRequest.keyframeMotion < KeyframeMotionID::firstNonGetUpAction && std::abs(engine.theInertialData.angle.y()) > 30_deg)
  {
    ANNOTATION("KeyframeMotionEngine:checkFinishedState", "ERROR Something went wrong");
    OUTPUT_ERROR("KeyframeMotionEngine:checkFinishedState: ERROR Something went wrong");
    tryCounter += 1;
    state = EngineState::breakUp;
  }
  else
    state = EngineState::waitForRequest;
}

void KeyframePhaseBase::doManipulateOwnState()
{
  int headSensorCounter = 0;
  headSensorCounter += engine.theKeyStates.pressed[KeyStates::Key::headFront] ? 1 : 0;
  headSensorCounter += engine.theKeyStates.pressed[KeyStates::Key::headMiddle] ? 1 : 0;
  headSensorCounter += engine.theKeyStates.pressed[KeyStates::Key::headRear] ? 1 : 0;
  // When getting up and at least 2 out of 3 head sensors are pressed, go directly into stand
  if(headSensorCounter >= 2 && engine.theFrameInfo.getTimeSince(abortWithHeadSensorTimestamp) > 1000
     && type == MotionPhase::getUp)
  {
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
    state = EngineState::working;
    safePreviousKeyframeData();
    // SET Stand motion
    tooManyTries = false;
    wasHelpMe = false;
    waitForFallenCheck = false;
    errorTriggered = false;
    lastKeyframe = Keyframe();
    currentKeyframe = Keyframe();
    lastMotion.reset();
    currentMotionBlock.clear();
    currentMotion = engine.motions[KeyframeMotionListID::stand];
    // TODO Set stand motion
    initKeyframeMotion(true);
    breakUpOn = false;
    wasInWaiting = false;
    isCurrentLineOver();
    setNextJoints();
    return;
  }
}
