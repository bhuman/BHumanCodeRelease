/**
 * @file KeyframePhaseInitLineImpl.cpp
 * This file declares functions for the KeyframePhase
 * Content: Implementation of all methods, that are needed to set up a to be executed keyframe
 * Also contains the update methods to update all reference values
 * @author Philip Reichenberg
 */

#include "KeyframePhaseBase.h"
#include "Modules/MotionControl/KeyframeMotionEngine/KeyframeMotionEngine.h"

void KeyframePhaseBase::selectMotionToBeExecuted()
{
  // Make sure both are reset after a breakUp
  currentMotionBlock.clear();
  currentKeyframe = Keyframe();
  // Follow up motion
  if(lastMotion.has_value())
  {
    for(const auto& nextPhase : lastMotion.value().startAsNextPhaseMotion)
    {
      if(checkConditions(nextPhase.conditions, nextPhase.isAndCondition))
      {
        currentMotion = engine.motions[nextPhase.startAsNextPhase];
        initKeyframeMotion(false);
        lastMotion.reset();
        return;
      }
    }
  }
  if(!currentMotion.startAsNextPhaseMotion.empty())
  {
    for(const auto& nextPhase : currentMotion.startAsNextPhaseMotion)
    {
      if(checkConditions(nextPhase.conditions, nextPhase.isAndCondition))
      {
        currentMotion = engine.motions[nextPhase.startAsNextPhase];
        initKeyframeMotion(false);
        return;
      }
    }
  }

  // Normal handling
  bool overrideMirror = false;
  if(currentKeyframeMotionRequest.keyframeMotion < KeyframeMotionID::firstNonGetUpAction)
  {
    overrideMirror = true;
    if((engine.safeUprightParameters.pitchDirection.isInside(engine.theInertialData.angle.y()) &&
        engine.safeUprightParameters.rollDirection.isInside(engine.theInertialData.angle.x())) ||
       forceStandMotionAfterPlaydead)
      currentMotion = engine.motions[KeyframeMotionListID::stand];
    else
      currentMotion = engine.motions[KeyframeMotionListID::recoverGeneric];
  }
  else
  {
    // Should std::size_t be used here?
    KeyframeMotionListID::KeyframeMotionListID toBeExecutedMotion = KeyframeMotionListID::KeyframeMotionListID(static_cast<std::size_t>(currentKeyframeMotionRequest.keyframeMotion) + static_cast<std::size_t>(KeyframeMotionListID::sitDown) - static_cast<std::size_t>(KeyframeMotionID::firstNonGetUpAction));
    currentMotion = engine.motions[toBeExecutedMotion];
    isMirror = currentKeyframeMotionRequest.mirror;
  }
  initKeyframeMotion(overrideMirror);
}

void KeyframePhaseBase::isCurrentLineOver()
{
  if(!errorTriggered)
  {
    waitTimestamp = engine.theFrameInfo.time;

    // Is the current keyframe over?
    const bool earlyBranchSwitch = checkEarlyBranch();
    if(((ratio >= 1.f && wasRatio100Percent) || earlyBranchSwitch) && // special front check, to prevent some rare falls
       (!engine.stepKeyframes || engine.theKeyStates.pressed[KeyStates::Key::chest]))
    {
      for(int i = 0; i < Joints::numOfJoints; ++i)
      {
        if(lastUnbalanced.angles[i] == JointAngles::off || lastUnbalanced.angles[i] == JointAngles::ignore || lastUnbalanced.stiffnessData.stiffnesses[i] == 0)
          lastUnbalanced.angles[i] = engine.theJointAngles.angles[i];
      }
      ratio = 0.f;
      if(engine.stepKeyframes || earlyBranchSwitch || checkConditions(currentKeyframe.waitConditions, true))
        initCurrentLine(true);
      else
        state = EngineState::waiting;
      // Init some values
      if(state == EngineState::working)
      {
        duration = currentKeyframe.duration;
        if(engine.stepKeyframes)
          duration = 2000;
        ratio = std::min(static_cast<float>(engine.theFrameInfo.getTimeSince(lineStartTimestamp) / duration), 1.f);
      }
    }
    // This check is done because we want to make sure to reach ratio == 1.f at least once
    wasRatio100Percent = ratio >= 1.f;
  }
  else
  {
    OUTPUT_ERROR("Error Happened!");
    state = EngineState::helpMeState;
  }
}

void KeyframePhaseBase::initKeyframeMotion(const bool overrideMirror)
{
  if(tooManyTries)
  {
    state = EngineState::helpMeState;
    return;
  }
  state = EngineState::working;
  breakUpOn = true;
  breakCounter = 0;
  isFirstLine = true;
  waitForFallenCheck = false;
  //odometry = engine.mofs[motionID].odometryOffset; TODO
  lineStartTimestamp = engine.theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);

  startJoints = lastJointRequest; //save the current joint angles
  lastUnbalanced.angles = lastJointRequest.angles;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(lastJointRequest.stiffnessData.stiffnesses[joint] == 0)
    {
      startJoints.angles[joint] = engine.theJointAngles.angles[joint];
      lastUnbalanced.angles[joint] = engine.theJointAngles.angles[joint];;
    }
  }
  lastOutputLineJointRequestAngles.angles = startJoints.angles;
  isInOptionalLine = false;

  // decide for the first get up try if it should be mirrored or not
  if(!didFirstGetUp)
  {
    didFirstGetUp = true;
    const int frame = static_cast<int>(engine.theFrameInfo.time / static_cast<unsigned int>(1000.f * Constants::motionCycleTime));
    getUpMirror = frame % 2;
  }

  if(overrideMirror)
  {
    isMirror = getUpMirror;
    getUpMirror = !getUpMirror;
  }

  // Is the requested motion broken?
  if(currentMotion.keyframes.empty() || engine.keyframeBlock[currentMotion.keyframes.front()].keyframes.empty())
  {
    OUTPUT_ERROR("This Motion does not have Motionlines!");
    errorTriggered = true;
  }
  else
    initCurrentLine(false);
  if(isMirror)
    odometry = Pose2f(-odometry.rotation, odometry.translation.x(), -odometry.translation.y());
}

void KeyframePhaseBase::safePreviousKeyframeData()
{
  lastKeyframe = currentKeyframe;
  lastKeyframeLineJointRequest.angles = lineJointRequest2Angles.angles;
}

void KeyframePhaseBase::initCurrentLine(const bool safeLastKeyframeInfo)
{
  if(safeLastKeyframeInfo)
    safePreviousKeyframeData();
  else
    lastKeyframeLineJointRequest.angles = startJoints.angles;

  checkOptionalLine();
  while(true)
  {
    bool newKeyframeBlock = false;
    while(currentMotionBlock.size() >= 1 && currentMotionBlock[0].keyframes.empty())
    {
      currentMotionBlock.erase(currentMotionBlock.begin());
      newKeyframeBlock = true;
    }
    if(currentMotionBlock.empty())  // init next block
    {
      while(true)
      {
        if(currentMotion.keyframes.empty())
          break;
        currentMotionBlock.push_back(engine.keyframeBlock[currentMotion.keyframes.front()]);
        currentMotion.keyframes.erase(currentMotion.keyframes.begin());
        if(!currentMotionBlock.empty() && !currentMotionBlock[0].keyframes.empty())
        {
          newKeyframeBlock = true;
          break;
        }
        else
          currentMotionBlock.clear();
      }
      if(currentMotion.keyframes.empty() && currentMotionBlock.empty())  // motion is over
      {
        state = EngineState::finished;
        checkFinishedState();
        return;
      }
    }
    if(newKeyframeBlock)
      setJointStiffnessBase();
    ASSERT(!currentMotionBlock.empty() && !currentMotionBlock[0].keyframes.empty());
    currentKeyframe = currentMotionBlock[0].keyframes[0];
    currentMotionBlock[0].keyframes.erase(currentMotionBlock[0].keyframes.begin());
    if(!checkEarlyBranch())
    {
      initBalancerValues(false);
      break;
    }
  }

  waitTimestamp = engine.theFrameInfo.time;
  lineStartTimestamp = engine.theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
  startJoints = lastUnbalanced;
  lastOutputLineJointRequestAngles.angles = startJoints.angles;
  for(unsigned int i = 0; i < numOfConditionVars; ++i)
    variableValuesCompare[i] = 0.f;
  failedWaitCounter = 0.f;
  initBalancerValues(false);
  jointCompensationReducer.fill(0.f);

  // In case one joint was used for balancing on the previous keyframe but now is not anymore, the last requested angle for these joints shall be the start joints for the new keyframe.
  // Otherwise the balancing value would be missing in these joints and they would jump and damage the gears.
  const std::vector<JointPair>& listY = currentKeyframe.balanceWithJoints.jointY;
  const std::vector<JointPair>& listX = currentKeyframe.balanceWithJoints.jointX;

  std::vector<Joints::Joint> jointListY;
  std::vector<Joints::Joint> jointListX;

  for(std::size_t i = 0; i < listY.size(); i++)
    jointListY.emplace_back(listY[i].joint);
  for(std::size_t i = 0; i < listX.size(); i++)
    jointListX.emplace_back(listX[i].joint);

  for(Joints::Joint joint : jointsBalanceY)
    if(!(std::find(jointListY.begin(), jointListY.end(), joint) != jointListY.end()))
      startJoints.angles[joint] = jointRequestOutput.angles[joint];
  for(Joints::Joint joint : jointsBalanceX)
    if(!(std::find(jointListX.begin(), jointListX.end(), joint) != jointListX.end()))
      startJoints.angles[joint] = jointRequestOutput.angles[joint];

  lastOutputLineJointRequestAngles.angles = startJoints.angles;
  jointsBalanceY = jointListY;
  jointsBalanceX = jointListX;

  // Is this the first keyframe of the motion? It must be a boolean and not a lineCounter == 0 check, because the first keyframe could be a conditional line.
  if(isFirstLine || forceBalancerInit)
  {
    isFirstLine = false;
    forceBalancerInit = false;
    initBalancerValues(true);
    for(std::size_t i = 0; i < pastJointAnglesAngles.capacity(); i++)
      pastJointAnglesAngles[i].angles.fill(JointAngles::off);
    jointDiff1Angles.angles.fill(0);
    jointDiffPredictedAngles.angles.fill(0);
  }

  ratio = 0.f;
  // Duration of the keyframe.
  duration = currentKeyframe.duration;
  updateLineValues();

  // Override reference com of last keyframe
  if(currentKeyframe.setLastCom)
  {
    lastGoal = comDiff;
    currentGoal = lastGoal + (currentKeyframe.goalCom - lastGoal) * ratio;
  }

  if(doSlowKeyframeAfterFall)
    duration = 2000;
  doSlowKeyframeAfterFall = false;
  balancerOn = currentKeyframe.balanceWithJoints.jointX.size() > 0 || currentKeyframe.balanceWithJoints.jointY.size() > 0;
  const bool energySavingLeg = (state == EngineState::waitForRequest && currentMotion.keyframeEndRequest.energySavingLegs == EnergySavingType::activeInWait) || currentMotion.keyframeEndRequest.energySavingLegs == EnergySavingType::activeAlways;
  const bool energySavingArms = (state == EngineState::waitForRequest && currentMotion.keyframeEndRequest.energySavingArms == EnergySavingType::activeInWait) || currentMotion.keyframeEndRequest.energySavingArms == EnergySavingType::activeAlways;

  if(engine.theEnergySaving.state == EnergySaving::EnergyState::working && !energySavingLeg && !energySavingArms)
    engine.theEnergySaving.reset();

  // Get current request
  lineJointRequest.angles[Joints::headYaw] = currentKeyframe.angles.head[0];
  lineJointRequest.angles[Joints::headPitch] = currentKeyframe.angles.head[1];
  for(unsigned int i = 0; i < 6; i++)
  {
    lineJointRequest.angles[Joints::lShoulderPitch + i] = currentKeyframe.angles.positions[KeyframeLimb::leftArm][i];
    lineJointRequest.angles[Joints::rShoulderPitch + i] = currentKeyframe.angles.positions[KeyframeLimb::rightArm][i];
    lineJointRequest.angles[Joints::lHipYawPitch + i] = currentKeyframe.angles.positions[KeyframeLimb::leftLeg][i];
    lineJointRequest.angles[Joints::rHipYawPitch + i] = currentKeyframe.angles.positions[KeyframeLimb::rightLeg][i];
  }
  setJointStiffnessKeyframe();
  if(isMirror)
  {
    JointAngles mirroredJoints;
    mirroredJoints.mirror(lineJointRequest);
    lineJointRequest.angles = mirroredJoints.angles;
  }

  // This is done because if a keyframe after the first uses off or ignore angles, the angles that are calculated will be off by up to 30 degree for the first frames
  // Also the difference check of set-joints and reached-joints are to high too.
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    if(lineJointRequest.angles[i] == JointAngles::off || lineJointRequest.stiffnessData.stiffnesses[i] == 0)
    {
      if(!wasInWaiting)
        lineJointRequest.angles[i] = lastUnbalanced.angles[i];
      else
        lineJointRequest.angles[i] = engine.theJointAngles.angles[i];
    }
    if(lineJointRequest.angles[i] == JointAngles::ignore)
    {
      lineJointRequest.angles[i] = engine.theJointRequest.angles[i];
    }
  }

  targetJoints = lineJointRequest;

  wasInWaiting = false;
  lineJointRequest2Angles.angles = lineJointRequest.angles;

  ASSERT(currentMotionBlock.size() > 0);
  currentKeyframe.interpolationType = currentMotionBlock[0].interpolationType;
}

void KeyframePhaseBase::updateLineValues()
{
  // For debugging on real robot
  if(engine.stepKeyframes)
    duration = 2000;
  // Fail Save
  if(duration < 1.f)
    duration = 1.f;
  ratio = static_cast<float>(engine.theFrameInfo.getTimeSince(lineStartTimestamp) / duration);
  if(duration - engine.theFrameInfo.getTimeSince(lineStartTimestamp) < 3.f)  // Optimization in case we are less than 3ms away from the last frame of the current keyframe.
    ratio = 1.f;
  if(ratio > 1.f)
    ratio = 1.f;

  // This low-pass filter is really important! Otherwise the D-part of the PID-Controller gets to high!
  goalGrowth = 0.75f * goalGrowth + 0.25f * (currentKeyframe.goalCom - lastGoal) * static_cast<float>(1000.f * Constants::motionCycleTime) / duration;
  currentGoal = lastGoal + (currentKeyframe.goalCom - lastGoal) * ratio;
  currentComDiffBaseForBalancing = lastComDiffBaseForBalancing + (engine.theKeyframeMotionParameters.balanceList[Phase(currentKeyframe.phase)].comDiffBase - lastComDiffBaseForBalancing) * ratio;
  currentTorsoAngleBreakUp = lastTorsoAngleBreakUp + (currentKeyframe.torsoAngleBreakUpEnd - lastTorsoAngleBreakUp) * ratio;
}

void KeyframePhaseBase::initBalancerValues(bool calculateCurrentNew)
{
  const float mirrorFactor = isMirror ? -1.f : 1.f;

  // This is done, because if we were in a retry state, we cant take the last currentValues
  // and need to calculate them new, because our lineCounter jumped backwards.
  if(calculateCurrentNew)  // no check needed?
  {
    currentGoal = currentKeyframe.goalCom;
    currentGoal.y() *= mirrorFactor;
    currentComDiffBaseForBalancing = engine.theKeyframeMotionParameters.balanceList[Phase(currentKeyframe.phase)].comDiffBase;
  }
  if(currentKeyframe.torsoAngleBreakUpStart.has_value())
  {
    // mirror to get the original value
    if(isMirror)
      currentTorsoAngleBreakUp.rollDirection = Rangea(currentTorsoAngleBreakUp.rollDirection.max * mirrorFactor, currentTorsoAngleBreakUp.rollDirection.min * mirrorFactor);
    // assign start value
    currentTorsoAngleBreakUp.assignDirectionValue(currentKeyframe.torsoAngleBreakUpStart.value());
    // mirror to get the value for the mirrored motion
    if(isMirror)
      currentTorsoAngleBreakUp.rollDirection = Rangea(currentTorsoAngleBreakUp.rollDirection.max * mirrorFactor, currentTorsoAngleBreakUp.rollDirection.min * mirrorFactor);
  }

  // Make sure all JointAngle::ignore values are replaced
  currentTorsoAngleBreakUp.assignIgnoreValue(2000_deg);
  // Replace ignore end values with current ones
  currentKeyframe.torsoAngleBreakUpEnd.replaceIgnoreDirectionValue(currentTorsoAngleBreakUp);

  lastGoal = currentGoal;
  lastComDiffBaseForBalancing = currentComDiffBaseForBalancing;
  lastTorsoAngleBreakUp.assignDirectionValue(currentTorsoAngleBreakUp);

  // Mirror all values if needed
  if(isMirror)
  {
    currentKeyframe.goalCom.y() *= mirrorFactor;
    currentKeyframe.torsoAngleBreakUpEnd.rollDirection = Rangea(currentKeyframe.torsoAngleBreakUpEnd.rollDirection.max * mirrorFactor, currentKeyframe.torsoAngleBreakUpEnd.rollDirection.min * mirrorFactor);
  }
}

void KeyframePhaseBase::setJointStiffnessBase()
{
  ASSERT(!currentMotionBlock.empty());
  for(unsigned int i = 0; i < 2; i++)
    if(currentMotionBlock[0].baseLimbStiffness[0] != -1)
      targetJoints.stiffnessData.stiffnesses[i] = currentMotionBlock[0].baseLimbStiffness[0];
  for(unsigned int i = 0; i < 6; i++)
  {
    for(unsigned int j = 0; j < 4; j++)
      if(currentMotionBlock[0].baseLimbStiffness[1 + j] != -1)
        targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::lShoulderPitch + i + 6 * j)) : Joints::lShoulderPitch + i + 6 * j] = currentMotionBlock[0].baseLimbStiffness[1 + j];
    if(engine.stepKeyframes)
      for(size_t i = 0; i < Joints::numOfJoints; i++)
        targetJoints.stiffnessData.stiffnesses[i] = std::min(engine.maxStiffnessDebugMode, targetJoints.stiffnessData.stiffnesses[i]);
    lineJointRequest.stiffnessData = targetJoints.stiffnessData;
  }
}

void KeyframePhaseBase::setJointStiffnessKeyframe()
{
  // set single motor stiffness change
  for(std::size_t i = 0; i < currentKeyframe.singleMotorStiffnessChange.size(); i++)
    targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(currentKeyframe.singleMotorStiffnessChange[i].joint) : currentKeyframe.singleMotorStiffnessChange[i].joint] =
      !engine.stepKeyframes ? currentKeyframe.singleMotorStiffnessChange[i].s : std::min(engine.maxStiffnessDebugMode, currentKeyframe.singleMotorStiffnessChange[i].s);
  lineJointRequest.stiffnessData = targetJoints.stiffnessData;
}

void KeyframePhaseBase::updateSensorArray()
{
  lastTorsoAngle.push_front(engine.theInertialData.angle.head<2>());

  const Vector2a vel = lastTorsoAngle[0] - lastTorsoAngle[1];
  const Vector2a acc = vel - (lastTorsoAngle[1] - lastTorsoAngle[2]);

  fluctuation = Vector2a(std::abs(acc.x()), std::abs(acc.y())) + Vector2a(std::abs(vel.x()), std::abs(vel.y()));

  updateLineValues();
  calculateCOMInSupportPolygon();
}
