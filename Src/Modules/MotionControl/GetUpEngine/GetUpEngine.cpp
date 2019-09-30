/*
 * @file GetUpEngine.cpp
 * @author <A href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</A>
 */

#include "GetUpEngine.h"

using namespace std;

GetUpEngine::GetUpEngine()
{
  state = EngineStates::off;
  motionID = GetUpMotion::stand;
  lineStartTimestamp = 0;
  lastNotActiveTimestamp = 0;
  waitTimestamp = 0;
  getUpFinishedTimStamp = 0;
  helpMeTimestamp = 0;
  breakUpTimestamp = 0;
  lastPIDCom = 0;
  retryTimestamp = 0;
  doBalanceOutTimestamp = 0;
  abortWithHeadSensorTimestamp = 0;

  breakCounter = 0;
  retryTime = 0;
  lineCounter = 0;
  maxCounter = 0;
  tryCounter = 0;

  startJoints = JointAngles();

  lastUnbalanced = JointRequest();
  targetJoints = JointRequest();
  jointDif1Angles.angles = JointRequest().angles;
  lineJointRequest = JointRequest();

  jointDifPredictedAngles.angles = JointRequest().angles;
  lastLineJointRequestAngles.angles = JointRequest().angles;

  jointsBalanceY = jointsBalanceX = std::vector<Joints::Joint>();

  lastBackwardAngleBreakUp = lastForwardAngleBreakUp = currentBackwardAngleBreakUp = currentForwardAngleBreakUp = Vector2a::Zero();
  lastBackwardCOMDif = lastForwardCOMDif = currentBackwardCOMDif = currentForwardCOMDif = Vector2f::Zero();

  lastGoal = Vector2f(0.f, 0.f);
  goalGrowth = Vector2f(0.f, 0.f);
  currentGoal = Vector2f(0.f, 0.f);

  oldCom = Vector2f(0.f, 0.f);
  oldCom2 = Vector2f(0.f, 0.f);

  valPID_I = Vector2f(0.f, 0.f);
  valPID_D = Vector2f(0.f, 0.f);
  valPID_P = Vector2f(0.f, 0.f);

  comDif = Vector2f(0.f, 0.f);
  balanceValue = Vector2f(0.f, 0.f);
  fluctuation = Vector2a::Zero();

  ratio = 1.f;
  duration = 0.f;
  framesTillEnd = 1.f;
  failedWaitCounter = 0;
  retryMotionCounter = 0;
  pidDForward = 0.f;

  fastRecover = false;
  specialActionCheck = false;
  armLeftCheck = false;
  armRightCheck = false;
  sideCheck = false;
  waitForFallenCheck = false;
  errorTriggered = false; //If an error occurred -> true
  isContinueTo = false;
  tooManyTries = false;
  wasHelpMe = false;
  doBalance = false;
  wasRatio100Percent = false;
  initPhase = false;
  balancerOn = false;
  isFirstLine = false;
  breakUpOn = false;
  isMirror = false;
  isInOptionalLine = false;
  wasInWaiting = false;
  getUpMirror = false;
  didFirstGetUp = false;
  stepKeyframes = false;
  isLeavingPossible = false;

  //init ringbuffer
  lastTorsoAngle.push_front(Vector2a::Zero());
  lastTorsoAngle.push_front(Vector2a::Zero());
  lastTorsoAngle.push_front(Vector2a::Zero());
}

void GetUpEngine::update(GetUpEngineOutput& output)
{
#ifndef NDEBUG
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lAnklePitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rAnklePitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lKneePitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rKneePitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lHipPitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rHipPitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lHipYawPitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lShoulderPitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rShoulderPitch");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lAnkleRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rAnkleRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lShoulderRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rShoulderRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lElbowRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rElbowRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lElbowYaw");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rElbowYaw");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lWristYaw");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rWristYaw");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:lHipRoll");
  DECLARE_PLOT("module:GetUpEngine:jointDif1:rHipRoll");

  DECLARE_PLOT("module:GetUpEngine:pid:p");
  DECLARE_PLOT("module:GetUpEngine:pid:i");
  DECLARE_PLOT("module:GetUpEngine:pid:d");
  DECLARE_PLOT("module:GetUpEngine:pid:balance");

  DECLARE_PLOT("module:GetUpEngine:pid:borderF");
  DECLARE_PLOT("module:GetUpEngine:pid:borderB");

  DECLARE_PLOT("module:GetUpEngine:pid:com");
  DECLARE_PLOT("module:GetUpEngine:pid:angleY");
  DECLARE_PLOT("module:GetUpEngine:pid:angleX");

  DEBUG_RESPONSE_ONCE("module:GetUpEngine:setSteppingKeyframes")
    stepKeyframes = true;

#endif
  output.odometryOffset = Pose2f();
  checkOwnState(output);
  output.currentLine.ratio = ratio;
  output.currentLine.state = state;
  output.recoverArmLeftDone = armLeftCheck;
  output.recoverArmRightDone = armRightCheck;
  output.recoverSideDone = sideCheck;
  output.errorTriggered = errorTriggered;
  output.currentLine.lineCounter = lineCounter;
  output.currentLine.maxCounter = maxCounter;
  output.currentLine.isMirror = isMirror;
  output.tryCounter = tryCounter;
  output.currentLine.isInOptionalLine = isInOptionalLine;
  output.lineInfo.balanceValue = balanceValue;
  output.lineInfo.comDif = comDif;
  isLeavingPossible = output.isLeavingPossible;
  odometry = output.odometryOffset;
  framesTillEnd = output.currentLine.framesTillEnd;
}

void GetUpEngine::update(GetUpEngineOutputLog& output)
{
  output.isLeavingPossible = isLeavingPossible;
  output.odometryOffset = odometry;
  output.tryCounter = tryCounter;
  output.currentLine.state = state;
  output.currentLine.name = motionID;
  output.currentLine.lineCounter = lineCounter;
  output.currentLine.maxCounter = maxCounter;
  output.currentLine.framesTillEnd = framesTillEnd;
  output.currentLine.isMirror = isMirror;
  output.currentLine.isInOptionalLine = isInOptionalLine;
  output.lineInfo.balanceValue = balanceValue;
  output.lineInfo.comDif = comDif;
  output.recoverArmLeftDone = armLeftCheck;
  output.recoverArmRightDone = armRightCheck;
  output.recoverSideDone = sideCheck;
  output.errorTriggered = errorTriggered;
  output.currentLine.ratio = ratio;
  output.waitTime = theGetUpEngineOutput.waitTime;
}

void GetUpEngine::checkOwnState(GetUpEngineOutput& output)
{
  //Is the getUpEngine active?
  if(theLegMotionSelection.ratios[MotionRequest::getUp] > 0.f)
  {
    //If the getUpEngine went active AND the first keyframe needs balancing, then we would need to wait 3 motion frames to make sure that the last 3 calculated "com" values
    //(comDif, oldCom, oldCom2) are from the current get up try. But because there is always at least 1 keyframe before a keyframe that needs balancing, this "problem" will never happen.
    updateSensorArray(output);
    //check for special events
    doManipulateOwnState(output);

    //is this the first frame the getUpEngine is on?
    if(state == EngineStates::off)
    {
      output.isLeavingPossible = false;
      //If there is a problem with the sensory update, wait until it is fixed (V6 head flex problem)
      if(theGyroOffset.gyroIsStuck)
        return;
      retryMotionCounter = 0;
      //Init pastJointAngles. This is done to prevent wrong calculated joint differences in calculateJointDifference
      for(std::size_t i = 0; i < pastJointAnglesAngles.size(); i++)
        pastJointAnglesAngles[i].angles = theJointAngles.angles;
      //Init jointDif. This is done to prevent wrong calculated joint differences in calculateJointDifference
      for(std::size_t i = 0; i < jointDif1Angles.angles.size(); i++)
      {
        jointDif1Angles.angles[i] = 0.f;
        jointDifPredictedAngles.angles[i] = 0.f;
        lastLineJointRequestAngles.angles[i] = JointAngles::off;
      }
      tryCounter = 0;
      //Fail save. If there is a bug, then the geUpEngine wont execute the angles from the last time the getUpEngine was on.
      for(std::size_t i = 0; i < output.angles.size(); i++)
        output.angles[i] = JointAngles::ignore;
      lastNotActiveTimestamp = theFrameInfo.time;
      waitForFallenCheck = true;
      state = EngineState::decideAction;
    }
    //Wait until we can start a new get up try.
    if(state == EngineStates::breakUp)
      waitForFallen(output);
    //Decide what to do
    if(state == EngineStates::decideAction)
    {
      //Check which motion shall be executed
      checkRF(output);
      int counter = 0;
      //This is done because if the set motion has no lines (which should only happen due to a human error), we want to skip this motion
      while(maxCounter == 0 && counter < 8)
      {
        state = EngineState::decideAction;
        checkRF(output);
        counter++;
        errorTriggered = false;
      }
      //We found no valid get up motion.
      if(counter == 8)
        state = EngineState::helpMeState;
    }
    //Check if we waited long enough and can continue our motion.
    if(state == EngineStates::waiting)
    {
      int waitTime = mofs[motionID].lines[lineCounter].waitConditions.size() == 1 ? mofs[motionID].lines[lineCounter].waitConditions[0].maxWaitTime : 1000;
      output.waitTime = theFrameInfo.getTimeSince(waitTimestamp);
      //Waiting over?
      if(checkConditions(mofs[motionID].lines[lineCounter].waitConditions, true, output) || theFrameInfo.getTimeSince(waitTimestamp) >= waitTime || failedWaitCounter > 8.f)
      {
        wasInWaiting = true;
        lineStartTimestamp = theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
        state = EngineStates::working;
        initCurrentLine(output);
        updateLineValues(output);
      }
      //We are in break up range and need to break the motion?
      else if(!isInBreakUpRange(output))
      {
        //We are still in a waiting time. We set the next joints so we can still balance while we wait.
        ratio = 1.f;
        setNextJoints(output);
      }
    }
    //Check retry state. This is an optimization so we can retry motions without using the tryCounter.
    if(state == EngineState::retryState)
    {
      //Wait time over?
      if(theFrameInfo.getTimeSince(retryTimestamp) > retryTime)
      {
        //Init all the motion stuff so we can retry the motion from a given line.
        state = EngineState::working;
        if(lineCounter == 0)
          initGetUpMotion(output);
        else
        {
          initBalancerValues(output, true);
          startJoints = lastUnbalanced;
          lastLineJointRequestAngles.angles = startJoints.angles;
          initCurrentLine(output);
          lineStartTimestamp = theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
          ratio = 0.f;
          updateLineValues(output);
        }
        duration = 1000;
      }
    }
    //We are at the end of the get up motion and are standing. Keep balancing.
    if(state == EngineState::balanceOut)
    {
      doBalanceOut(output);
    }
    //Check if we can continue the current motion and set the next joints.
    if(state == EngineStates::recoverFallen || state == EngineStates::working)
    {
      //calc current ratio;
      isCurrentLineOver(output);
      //Need to break the motion?
      isInBreakUpRange(output);
      //continue motion
      if(state == EngineStates::working || state == EngineState::recoverFallen)
        setNextJoints(output);
    }
    if(state == EngineState::finished || state == EngineState::finishedRecover)
      checkFinishedState(output);
    //We have no get up trys left.
    if(state == EngineStates::helpMeState)
      doHelpMeStuff(output);
  }
  //The getUpEngine should be off.
  else
  {
    if(state != EngineState::off)
      shutDownGetUpEngine(output);
  }
}

//The pastJointAngles values need to reset for every motion try
//also it needs to wait pastJointAngles.size() frames until it can start the checks. Otherwise the joint angle differences will be large, because
//the sensors are really slow
void GetUpEngine::calculateJointDifference(GetUpEngineOutput& output)
{
  //Because calculateJointDifference can be executed while in balanceOutState and in that state we are already shutting down the getUpEngine,
  //we dont want to check the Joints if that is the case.
  if(!output.isLeavingPossible)
  {
    for(std::size_t i = 0; i < pastJointAnglesAngles.size(); i++)
    {
      if(i + 1 < pastJointAnglesAngles.size())
        pastJointAnglesAngles[i] = pastJointAnglesAngles[i + 1];
      else
      {
        for(std::size_t j = 0; j < pastJointAnglesAngles[0].angles.size(); j++)
        {
          output.angles[j] != JointAngles::ignore&& output.angles[j] != JointAngles::off
          ? (pastJointAnglesAngles[i].angles[j] = output.angles[j])
          : (pastJointAnglesAngles[i].angles[j] = theJointAngles.angles[j]);
        }
      }
    }
    //Every Joint, that has a joint compensation shall be checked if the jointDif shall be predicted 3 frames into the future.
    std::vector<bool> predcit = std::vector<bool>(Joints::numOfJoints);
    if(!mofs[motionID].lines[lineCounter].jointCompensation.empty())
      for(JointCompensationParams list : mofs[motionID].lines[lineCounter].jointCompensation[0].jointCompensationParams)
        predcit[list.jointDelta] = list.predictJointDif;

    //Calculated current joint difference of "set" angles and "reached" angles.
    for(std::size_t i = 0; i < pastJointAnglesAngles[0].angles.size(); i++)
    {
      Angle dif = pastJointAnglesAngles[0].angles[i] - theJointAngles.angles[i];
      Angle prevDif = jointDif1Angles.angles[i];
      jointDif1Angles.angles[i] = pastJointAnglesAngles[0].angles[i] != -2000_deg
                                  ? 0.6f * prevDif + 0.4f * dif
                                  : 0.f;
      jointDifPredictedAngles.angles[i] = jointDif1Angles.angles[i];

      if(predcit[i])
      {
        jointDifPredictedAngles.angles[i] += 3.f * (jointDif1Angles.angles[i] - prevDif);
        jointDifPredictedAngles.angles[i] = std::abs(jointDif1Angles.angles[i]) < std::abs(jointDifPredictedAngles.angles[i]) ? jointDif1Angles.angles[i] : jointDifPredictedAngles.angles[i];
      }
    }
    //in addJointCompensation the joints are mirrored when the motion is mirrored. Thats why we make sure rHYP and lHYP are the same
    jointDif1Angles.angles[Joints::rHipYawPitch] = jointDif1Angles.angles[Joints::lHipYawPitch];
    jointDifPredictedAngles.angles[Joints::rHipYawPitch] = jointDifPredictedAngles.angles[Joints::lHipYawPitch];

    PLOT("module:GetUpEngine:jointDif1:lAnklePitch", jointDifPredictedAngles.angles[Joints::lAnklePitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rAnklePitch", jointDifPredictedAngles.angles[Joints::rAnklePitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lKneePitch", jointDifPredictedAngles.angles[Joints::lKneePitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rKneePitch", jointDifPredictedAngles.angles[Joints::rKneePitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lHipPitch", jointDifPredictedAngles.angles[Joints::lHipPitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rHipPitch", jointDifPredictedAngles.angles[Joints::rHipPitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lHipYawPitch", jointDifPredictedAngles.angles[Joints::lHipYawPitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lShoulderPitch", jointDifPredictedAngles.angles[Joints::lShoulderPitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rShoulderPitch", jointDifPredictedAngles.angles[Joints::rShoulderPitch].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lAnkleRoll", jointDifPredictedAngles.angles[Joints::lAnkleRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rAnkleRoll", jointDifPredictedAngles.angles[Joints::rAnkleRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lShoulderRoll", jointDifPredictedAngles.angles[Joints::lShoulderRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rShoulderRoll", jointDifPredictedAngles.angles[Joints::rShoulderRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lElbowRoll", jointDifPredictedAngles.angles[Joints::lElbowRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rElbowRoll", jointDifPredictedAngles.angles[Joints::rElbowRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lElbowYaw", jointDifPredictedAngles.angles[Joints::lElbowYaw].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rElbowYaw", jointDifPredictedAngles.angles[Joints::rElbowYaw].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lWristYaw", jointDifPredictedAngles.angles[Joints::lWristYaw].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rWristYaw", jointDifPredictedAngles.angles[Joints::rWristYaw].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:lHipRoll", jointDifPredictedAngles.angles[Joints::lHipRoll].toDegrees());
    PLOT("module:GetUpEngine:jointDif1:rHipRoll", jointDifPredictedAngles.angles[Joints::rHipRoll].toDegrees());
  }
}

void GetUpEngine::checkRF(GetUpEngineOutput& output)
{
  isMirror = false;
  const float& bodyAngleY = theInertialData.angle.y();
  const float& bodyAngleX = theInertialData.angle.x();

  //First check if robot did a special action
  if(!specialActionCheck && std::abs(bodyAngleY) < 30_deg && theMotionInfo.motion == MotionRequest::specialAction)
  {
    specialActionCheck = true;
    //Robot is sitting
    if(theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::sitDown)
    {
      state = EngineStates::working;
      motionID = GetUpMotion::stand;
    }
  }
  //Did the getUpEngine went activ while the robot is still standing? -> directly go to stand and leave.
  //This is done, because the fallDownStateProvider will activate the fall state of the
  //robot, when he is oscilate while leaving the getUpEngine and interpolating into stand.
  //This Bug occurs in the 1vs1Demo in PenaltyShootout too, everytime a robot is in set and gets picked up.
  if(theFrameInfo.getTimeSince(lastNotActiveTimestamp) < 5 && state == EngineState::decideAction)
  {
    if(std::fabs(bodyAngleY) < 15_deg
       && std::fabs(bodyAngleX) < 15_deg
       && theJointAngles.angles[Joints::lKneePitch] < 60_deg
       && theJointAngles.angles[Joints::rKneePitch] < 60_deg
       && theRobotModel.soleLeft.translation.z() < -200.f
       && theRobotModel.soleRight.translation.z() < -200.f)
    {
      state = EngineStates::working;
      motionID = GetUpMotion::stand;
      doFastStand = true;
    }
  }
  if(theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::allBroken)
  {
    state = EngineState::helpMeState;
    motionID = GetUpMotion::doNothing;
  }
  //Is Robot in a splitting or sitting position?
  if(state == EngineState::decideAction && std::abs(bodyAngleX) < 35_deg && std::abs(bodyAngleY) < 35_deg)
  {
    state = EngineStates::working;
    motionID = GetUpMotion::sit;
    if(GetUpMotions::GetUpMotion(motionID) == GetUpMotions::doNothing)
      state = EngineStates::decideAction;
  }
  //Is Robot lying on side?
  if(!sideCheck && state == EngineStates::decideAction)
  {
    sideCheck = true;
    //NAO lies on his right arm
    if(bodyAngleX > 40_deg)
    {
      state = EngineStates::recoverFallen;
      //recover to back?
      if(bodyAngleY < -10_deg)
        motionID = GetUpMotion::recoverFromSideBack;
      //recover to front!
      else
        motionID = GetUpMotion::recoverFromSideFront;
    }
    else if(bodyAngleX < -40_deg)
    {
      state = EngineStates::recoverFallen;
      //recover to back?
      isMirror = true;
      if(bodyAngleY < -10_deg)
        motionID = GetUpMotion::recoverFromSideBack;
      //recover to front!
      else
        motionID = GetUpMotion::recoverFromSideFront;
    }
    //NAO lies on his left arm

    if(GetUpMotions::GetUpMotion(motionID) == GetUpMotions::doNothing)
      state = EngineStates::decideAction;
  }

  //Robot is lying on ground. Do a fast recover to see if all problem can be solved by that
  if(!fastRecover && state == EngineStates::decideAction)
  {
    fastRecover = true;
    state = EngineStates::recoverFallen;
    motionID = GetUpMotion::recoverFast;
    if(GetUpMotions::GetUpMotion(motionID) == GetUpMotions::doNothing)
      state = EngineStates::decideAction;
  }
  //Is Robot lying on his arms?
  if((!armLeftCheck || !armRightCheck) && state == EngineStates::decideAction)
  {
    armLeftCheck = true;
    armRightCheck = true;
    //lying on front
    if(bodyAngleY > 45_deg)
    {
      //check both arms
      if((theRobotModel.limbs[Limbs::wristLeft].translation.y() < 90.f && theJointAngles.angles[Joints::lShoulderPitch] > 0_deg)
         || (theRobotModel.limbs[Limbs::wristRight].translation.y() > -90.f && theJointAngles.angles[Joints::rShoulderPitch] > 0_deg))
      {
        isMirror = false;
        state = EngineStates::recoverFallen;
        motionID = GetUpMotion::recoverArmLeftFrontLyingOn;
      }
      if(GetUpMotions::GetUpMotion(motionID) == GetUpMotions::doNothing)
        state = EngineStates::decideAction;
    }
  }
  //Robot can start a get up motion.
  if(state == EngineStates::decideAction)
  {
    //Start front motion
    if(bodyAngleY > 65_deg)
    {
      state = EngineStates::working;
      tryCounter >= maxTryCounter ? tooManyTries = true : motionID = GetUpMotion::front;
    }
    //Start back motion
    else if(bodyAngleY < -65_deg)
    {
      state = EngineStates::working;
      tryCounter >= maxTryCounter ? tooManyTries = true : motionID = GetUpMotion::back;
    }
    //We are too long in deciding action. To prevent a softlock, just start a get up motion
    else if(theFrameInfo.getTimeSince(lastNotActiveTimestamp) > 300)
    {
      //Start front motion
      if(bodyAngleY < 0)
      {
        state = EngineStates::working;
        tryCounter >= maxTryCounter ? tooManyTries = true : motionID = GetUpMotion::back;
      }
      //Start back motion
      else
      {
        state = EngineStates::working;
        tryCounter >= maxTryCounter ? tooManyTries = true : motionID = GetUpMotion::front;
      }
    }
    //We are upright enough to go into standing.
    else if(std::abs(bodyAngleY) < 30_deg && std::abs(bodyAngleX) < 20_deg)
    {
      state = EngineStates::working;
      motionID = GetUpMotion::stand;
    }
    //else chill on ground like a boss
  }
  if(state == EngineStates::working || state == EngineState::recoverFallen)
    initGetUpMotion(output);
  else if(state == EngineState::helpMeState)
    doHelpMeStuff(output);
  //Should never trigger, but better save than sorry.
  else if(state == EngineState::decideAction)
  {
    SystemCall::say("This Code should not be executed");
    ANNOTATION("GetUpEngine", "This Code should not be executed");
    for(std::size_t i = 0; i < output.angles.size(); i++)
      output.angles[i] = JointAngles::ignore;
  }
}

bool GetUpEngine::isInBreakUpRange(GetUpEngineOutput& output)
{
  if(stepKeyframes)
    return false;
  Angle yTorso = theInertialData.angle.y();
  Angle xTorso = theInertialData.angle.x();

  //Need to break?
  bool retryFront = checkRetryForFront(output);
  if(((theFilteredCurrent.legMotorMalfunction && motorMalfunctionBreakUp) || retryFront
      || yTorso > currentForwardAngleBreakUp.x()
      || yTorso < currentBackwardAngleBreakUp.x()
      || xTorso > currentForwardAngleBreakUp.y()
      || xTorso < currentBackwardAngleBreakUp.y())
     && breakUpOn && output.isLeavingPossible != true)
  {
    if(!checkMotionSpecificActions(output))
    {
      //Can we do a get up try afterwards?
      if(tryCounter >= maxTryCounter)
        state = EngineState::helpMeState;
      else
      {
        retryMotionCounter += retryFront ? 1 : 0;
        if(SystemCall::getMode() != SystemCall::simulatedRobot && !retryFront)
          tryCounter += 1;
        state = EngineStates::breakUp;
        waitForFallenCheck = true;
        //OUTPUT_TEXT("Robot needs to break!");
        if(SystemCall::getMode() != SystemCall::logFileReplay)
          SystemCall::say("Abort");
        for(std::size_t i = 0; i < output.angles.size(); i++)
          output.stiffnessData.stiffnesses[i] = 20;
      }
      return true;
    }
  }
  return false;
}

bool GetUpEngine::checkMotionSpecificActions(GetUpEngineOutput& output)
{
  //Do we have a retry set up?
  if(Phase(mofs[motionID].lines[lineCounter].phase) >= Phase::Lying&& Phase(mofs[motionID].lines[lineCounter].phase) <= Phase::ReduceVel&& breakCounter < motionSpecificRetrys)
  {
    retryTimestamp = theFrameInfo.time;
    breakCounter += 1;
    state = EngineState::retryState;
    retryTime = 1000;
    if(motionID == GetUpMotion::back)
      lineCounter = lineCounter > 1 ? 1 : 0;
    else
      return false;
    return true;
  }
  return false;
}

bool GetUpEngine::checkRetryForFront(GetUpEngineOutput& output)
{
  if(motionID == GetUpMotion::front && lineCounter == 2 && ratio < 0.1f && retryMotionCounter < motionSpecificRetrysFront)
  {
    //Check if the ShoulderRolls could not move and are behinde too much
    bool leftArmBehind = theJointAngles.angles[Joints::lShoulderPitch] > shoulderPitchThreshold && brokenJointData[3] <= std::fabs(jointDif1Angles.angles[3].toDegrees()) ? true : false;
    bool rightArmBehind = theJointAngles.angles[Joints::rShoulderPitch] > shoulderPitchThreshold && brokenJointData[9] <= std::fabs(jointDif1Angles.angles[9].toDegrees()) ? true : false;
    return leftArmBehind || rightArmBehind;
  }
  return false;
}

void GetUpEngine::isCurrentLineOver(GetUpEngineOutput& output)
{
  if(!errorTriggered)
  {
    //Is the current keyframe over?
    if(ratio >= 1.f && wasRatio100Percent && (!stepKeyframes || theKeyStates.pressed[KeyStates::Key::chest]))
    {
      lineStartTimestamp = theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
      startJoints = lastUnbalanced;
      lastLineJointRequestAngles.angles = startJoints.angles;
      ratio = 0.f;
      //Do we need to do a wait time?
      if(stepKeyframes || checkConditions(mofs[motionID].lines[lineCounter].waitConditions, true, output))
        initCurrentLine(output);
      else
      {
        state = EngineStates::waiting;
        waitTimestamp = theFrameInfo.time;
        //if(SystemCall::getMode() == SystemCall::logFileReplay)
        // SystemCall::say("Wait");
      }
      //Init some values
      if(state == EngineState::working)
      {
        duration = mofs[motionID].lines[lineCounter].duration;
        if(stepKeyframes)
          duration = 2000;
        output.currentLine.framesTillEnd = (duration - theFrameInfo.getTimeSince(lineStartTimestamp)) / static_cast<int>(1000.f * Constants::motionCycleTime);
        ratio = std::min(static_cast<float>(theFrameInfo.getTimeSince(lineStartTimestamp) / duration), 1.f);
      }
    }
    //This check is done because we want to make sure to reach ratio == 1.f at least once
    wasRatio100Percent = ratio >= 1.f;
  }
  else
    OUTPUT_ERROR("Error Happend!");
}

void GetUpEngine::initGetUpMotion(GetUpEngineOutput& output)
{
  ANNOTATION("GetUpEngine", motionID);
  breakUpOn = true;
  breakCounter = 0;
  isFirstLine = true;
  if(tooManyTries)
    state = EngineStates::helpMeState;
  waitForFallenCheck = false;
  maxCounter = static_cast<int>(mofs[motionID].lines.size());
  output.odometryOffset = mofs[motionID].odometryOffset;
  lineCounter = -1;
  output.currentLine.name = motionID;
  if(motionID == GetUpMotion::doNothing)
    OUTPUT_ERROR("doNothing Motion");
  lineStartTimestamp = theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
  if(!isContinueTo)
  {
    startJoints = theJointAngles; //save the current joint angles
    lastUnbalanced.angles = theJointAngles.angles;
  }
  else
    startJoints = lastUnbalanced;
  lastLineJointRequestAngles.angles = startJoints.angles;
  isInOptionalLine = false;
  if(!isContinueTo)
    lastUnbalanced.angles = theJointAngles.angles;
  doBalance = mofs[motionID].balanceOut;

  //decide for the first get up try if it should be mirrored or not
  if(!didFirstGetUp)
  {
    didFirstGetUp = true;
    int frame = static_cast<int>(theFrameInfo.time / static_cast<unsigned int>(1000.f * Constants::motionCycleTime));
    getUpMirror = frame % 2;
  }

  //decide if the motion front and back should be mirrored or not
  if(!isContinueTo && (motionID == GetUpMotion::back || motionID == GetUpMotion::front))
  {
    if(theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::onlyMirrored
       || (theDamageConfigurationBody.brokenStandUp != DamageConfigurationBody::onlyNormal  && getUpMirror))
      isMirror = true;
    else
      isMirror = false;
    getUpMirror = !getUpMirror;
  }
  //Is the requested motion broken?
  if(lineCounter + 1 >= maxCounter)
  {
    OUTPUT_ERROR("This Motion does not have Motionlines!");
    errorTriggered = true;
  }
  else
    initCurrentLine(output);
  if(isMirror && motionID != GetUpMotion::recoverFast)
    output.odometryOffset = Pose2f(-output.odometryOffset.rotation, output.odometryOffset.translation.x(), -output.odometryOffset.translation.y());
  //If NAO is lying on the side and will move to the right, the odometryOffset is mirrored.
  //If NAO is not lying on the side, then the odometryOffset is 0
  //TODO: recoverFast has no odometry offset. This specialCase should be for recoverFromSideBack and recoverFromSideFront
  if(motionID == GetUpMotion::recoverFast)
  {
    if(std::abs(theInertialData.angle.x()) > 50_deg && theInertialData.angle.y() > 0_deg)
      output.odometryOffset = Pose2f(-output.odometryOffset.rotation, output.odometryOffset.translation.x(), -output.odometryOffset.translation.y());
    else if(theInertialData.angle.x() > -30_deg && theInertialData.angle.x() < 30_deg)
      output.odometryOffset = Pose2f();
  }
}

void GetUpEngine::initCurrentLine(GetUpEngineOutput& output)
{
  for(unsigned int i = 0; i < numOfConditionVars; ++i)
    variableValuesCompare[i] = 0.f;
  failedWaitCounter = 0.f;
  initBalancerValues(output, false);
  lineCounter += 1;
  checkOptionalLine(output);
  std::fill(jointCompensationReducer.begin(), jointCompensationReducer.end(), 0.f);

  if(lineCounter < maxCounter)
  {
    //In case one joint was used for balancing on the previous keyframe but now is not anymore, the last requested angle for these joints shall be the start joints for the new keyframe.
    //Otherwise the balancing value would be missing in these joints and they would jump and damage the gears.
    std::vector<JointFactor> listY = mofs[motionID].lines[lineCounter].balanceWithJoints.jointY;
    std::vector<JointFactor> listX = mofs[motionID].lines[lineCounter].balanceWithJoints.jointX;

    std::vector<Joints::Joint> jointListY;
    std::vector<Joints::Joint> jointListX;

    for(std::size_t i = 0; i < listY.size(); i++)
      jointListY.push_back(listY[i].joint);
    for(std::size_t i = 0; i < listX.size(); i++)
      jointListX.push_back(listX[i].joint);

    for(Joints::Joint joint : jointsBalanceY)
      if(!(std::find(jointListY.begin(), jointListY.end(), joint) != jointListY.end()))
        startJoints.angles[joint] = output.angles[joint];
    for(Joints::Joint joint : jointsBalanceX)
      if(!(std::find(jointListX.begin(), jointListX.end(), joint) != jointListX.end()))
        startJoints.angles[joint] = output.angles[joint];

    lastLineJointRequestAngles.angles = startJoints.angles;
    jointsBalanceY = jointListY;
    jointsBalanceX = jointListX;

    //Is this the first keyframe of the motion? It must be a boolean and not a lineCounter == 0 check, because the first keyframe could be a conditionale line.
    if(isFirstLine)
    {
      isFirstLine = false;
      if(!isContinueTo)
      {
        initBalancerValues(output, true);
        duration = mofs[motionID].lines[lineCounter].duration;
        updateLineValues(output);
        for(std::size_t i = 0; i < pastJointAnglesAngles.size(); i++)
          for(std::size_t j = 0; j < pastJointAnglesAngles[i].angles.size(); j++)
            pastJointAnglesAngles[i].angles[j] = convertToAngleSpecialCases(-2000.f);
        for(std::size_t i = 0; i < jointDif1Angles.angles.size(); i++)
        {
          jointDif1Angles.angles[i] = 0.f;
          jointDifPredictedAngles.angles[i] = 0.f;
        }
      }
      else
      {
        duration = mofs[motionID].lines[lineCounter].duration;
        updateLineValues(output);
        isContinueTo = false;
      }
    }

    //Override reference com of last keyframe
    if(mofs[motionID].lines[lineCounter].setLastCom)
    {
      lastGoal = comDif;
      currentGoal = lastGoal + (mofs[motionID].lines[lineCounter].goalCom - lastGoal) * ratio;
    }

    //Duration of the keyframe. If getUpEngine went active while robot is still standing, go directly into stand.
    duration = mofs[motionID].lines[lineCounter].duration;
    if(doFastStand && motionID == GetUpMotion::stand)
      duration = 300;
    doFastStand = false;
    ratio = 0.f;
    balancerOn = mofs[motionID].lines[lineCounter].balancerActive;
    //If the get up is nearly over, interpolate the head angle to the current behavior request.
    //This is done because of interpolation reasons from the motionSelector
    Phase phase = Phase(mofs[motionID].lines[lineCounter].phase);
    bool overrideHead = phase == Phase::Stand || phase == Phase::Sit;
    if(overrideHead)
    {
      lineJointRequest.angles[Joints::headYaw] += headInterpolation[phase] * (theHeadAngleRequest.pan - lineJointRequest.angles[Joints::headYaw]);
      lineJointRequest.angles[Joints::headPitch] += headInterpolation[phase] * (theHeadAngleRequest.tilt - lineJointRequest.angles[Joints::headPitch]);
    }
    else
    {
      lineJointRequest.angles[Joints::headYaw] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].head[0]);
      lineJointRequest.angles[Joints::headPitch] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].head[1]);
    }
    for(unsigned int i = 0; i < 6; i++)
    {
      lineJointRequest.angles[Joints::lShoulderPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].leftArm[i]);
      lineJointRequest.angles[Joints::rShoulderPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].rightArm[i]);
      lineJointRequest.angles[Joints::lHipYawPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].leftLeg[i]);
      lineJointRequest.angles[Joints::rHipYawPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter].rightLeg[i]);
    }
    setJointStiffness(output);
    if(isMirror)
    {
      JointRequest mirroredJoints;
      mirroredJoints.mirror(lineJointRequest);
      if(overrideHead)
      {
        mirroredJoints.angles[Joints::headYaw] = lineJointRequest.angles[Joints::headYaw];
        mirroredJoints.angles[Joints::headPitch] = lineJointRequest.angles[Joints::headPitch];
      }
      lineJointRequest.angles = mirroredJoints.angles;
    }
    targetJoints = lineJointRequest;

    //This is done because if a keyframe after the first uses off or ignore angles, the angles that are calculated will be off by up to 30 degree for the first frames
    //Also the difference check of set-joints and reached-joints are to high too.
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(targetJoints.angles[i] == JointAngles::off || targetJoints.angles[i] == JointAngles::ignore)
      {
        if(!wasInWaiting)
          lineJointRequest.angles[i] = lastUnbalanced.angles[i];
        else
          lineJointRequest.angles[i] = theJointAngles.angles[i];
      }
    }
    wasInWaiting = false;
    lineJointRequest2Angles.angles = lineJointRequest.angles;
  }
  else
  {
    if(state == EngineState::recoverFallen)
      state = EngineState::finishedRecover;
    else
      state = EngineState::finished;
  }
}

void GetUpEngine::updateLineValues(GetUpEngineOutput& output)
{
  //For debugging on real robot
  if(stepKeyframes)
    duration = 2000;
  //Fail Save
  if(duration == 0)
    duration = 1;
  ratio = static_cast<float>(theFrameInfo.getTimeSince(lineStartTimestamp) / duration);
  output.currentLine.framesTillEnd = (duration - theFrameInfo.getTimeSince(lineStartTimestamp)) / static_cast<int>(1000.f * Constants::motionCycleTime);
  if(duration - theFrameInfo.getTimeSince(lineStartTimestamp) < 3) //Optimization in case we are less than 3ms away from the last frame of the current keyframe.
    ratio = 1.f;
  if(ratio > 1.f)
    ratio = 1.f;

  //Magic parameters
  goalGrowth = 0.75f * goalGrowth + 0.25f * (mofs[motionID].lines[lineCounter].goalCom - lastGoal) * (static_cast<float>(1000.f * Constants::motionCycleTime) / duration);

  currentGoal = lastGoal + (mofs[motionID].lines[lineCounter].goalCom - lastGoal) * ratio;
  currentBackwardCOMDif = lastBackwardCOMDif + (theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].backwardCOMDif - lastBackwardCOMDif) * ratio;
  currentForwardCOMDif = lastForwardCOMDif + (theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].forwardCOMDif - lastForwardCOMDif) * ratio;
  currentBackwardAngleBreakUp = lastBackwardAngleBreakUp + (theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].backwardAngleBreakUp - lastBackwardAngleBreakUp) * ratio;
  currentForwardAngleBreakUp = lastForwardAngleBreakUp + (theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].forwardAngleBreakUp - lastForwardAngleBreakUp) * ratio;

  //Angle != 0
  if(mofs[motionID].clipAngle != 0)
  {
    //Is the current phase type registered to be clipped?
    if(std::find(clipBreakUpAngle.begin(), clipBreakUpAngle.end(), Phase(mofs[motionID].lines[lineCounter].phase)) != clipBreakUpAngle.end())
    {
      if(mofs[motionID].clipAngle > 0)
        currentForwardAngleBreakUp.x() = lastForwardAngleBreakUp.x() + (mofs[motionID].clipAngle - lastForwardAngleBreakUp.x()) * ratio;
      else
        currentBackwardAngleBreakUp.x() = lastBackwardAngleBreakUp.x() + (mofs[motionID].clipAngle - lastBackwardAngleBreakUp.x()) * ratio;
    }
  }

  float mirrorFactor = 1.f;
  if(isMirror)
    mirrorFactor = -1.f;

  currentBackwardCOMDif[1] *= mirrorFactor;
  currentForwardCOMDif[1] *= mirrorFactor;
  currentGoal[1] *= mirrorFactor;
}

void GetUpEngine::initBalancerValues(GetUpEngineOutput& output, bool calculateCurrentNew)
{
  //This is done, because if we were in a retry state, we cant take the last currentValues
  //and need to calculate them new, because our lineCounter jumped backwards.
  if(calculateCurrentNew && lineCounter >= 0 && lineCounter < maxCounter)
  {
    currentGoal = mofs[motionID].lines[lineCounter].goalCom;
    currentBackwardCOMDif = theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].backwardCOMDif;
    currentBackwardAngleBreakUp = theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].backwardAngleBreakUp;
    currentForwardCOMDif = theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].forwardCOMDif;
    currentForwardAngleBreakUp = theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].forwardAngleBreakUp;

    //Angle != 0
    if(mofs[motionID].clipAngle != 0)
    {
      //Is the current phase type registered to be clipped?
      if(std::find(clipBreakUpAngle.begin(), clipBreakUpAngle.end(), Phase(mofs[motionID].lines[lineCounter].phase)) != clipBreakUpAngle.end())
      {
        if(mofs[motionID].clipAngle > 0)
          currentForwardAngleBreakUp.x() = lastForwardAngleBreakUp.x() + (mofs[motionID].clipAngle - lastForwardAngleBreakUp.x()) * ratio;
        else
          currentBackwardAngleBreakUp.x() = lastBackwardAngleBreakUp.x() + (mofs[motionID].clipAngle - lastBackwardAngleBreakUp.x()) * ratio;
      }
    }
  }
  lastGoal = currentGoal;
  lastBackwardCOMDif = currentBackwardCOMDif;
  lastBackwardAngleBreakUp = currentBackwardAngleBreakUp;
  lastForwardCOMDif = currentForwardCOMDif;
  lastForwardAngleBreakUp = currentForwardAngleBreakUp;
}

void GetUpEngine::waitForFallen(GetUpEngineOutput& output)
{
  if(waitForFallenCheck && motionID != GetUpMotion::doNothing)
  {
    breakUpTimestamp = theFrameInfo.time;
    waitForFallenCheck = false;
    //Set stiffness off and set head into a save position
    for(std::size_t i = 0; i < output.angles.size(); i++)
      output.angles[i] = JointAngles::off;
    if(theInertialData.angle.y() > 0_deg)
      output.angles[1] = -25_deg;
    else
      output.angles[1] = 25_deg;

    output.angles[0] = 0_deg;
    output.stiffnessData.stiffnesses[0] = 50;
    output.stiffnessData.stiffnesses[1] = 50;
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
  }
  if(theFrameInfo.getTimeSince(breakUpTimestamp) > 1000)
  {
    state = EngineStates::decideAction;
    lastNotActiveTimestamp = theFrameInfo.time;
  }
  //We assume the head moved into a save position, so we can lower the stiffness
  if(theFrameInfo.getTimeSince(breakUpTimestamp) > 100)
  {
    output.stiffnessData.stiffnesses[0] = 20;
    output.stiffnessData.stiffnesses[1] = 20;
  }
  fastRecover = false;
  specialActionCheck = false;
  armLeftCheck = false;
  armRightCheck = false;
  sideCheck = false;
  //todo here is a softlock?

  //Too many tries or a malfunction in a motor was detected
  if(tryCounter >= maxTryCounter || (theFilteredCurrent.legMotorMalfunction && motorMalfunctionBreakUp))
    state = EngineState::helpMeState;
}

void GetUpEngine::doManipulateOwnState(GetUpEngineOutput& output)
{
  if(((theKeyStates.pressed[KeyStates::Key::headFront] && (theKeyStates.pressed[KeyStates::Key::headMiddle] || theKeyStates.pressed[KeyStates::Key::headRear]))
      || (theKeyStates.pressed[KeyStates::Key::headMiddle && theKeyStates.pressed[KeyStates::Key::headRear]])) && theFrameInfo.getTimeSince(abortWithHeadSensorTimestamp) > 1000)
  {
    abortWithHeadSensorTimestamp = theFrameInfo.time;
    motionID = GetUpMotion::stand;
    state = EngineState::working;
    lastUnbalanced.stiffnessData.stiffnesses = lineJointRequest.stiffnessData.stiffnesses;
    lastUnbalanced = output;
    initGetUpMotion(output);
    output.stiffnessData.stiffnesses = lastUnbalanced.stiffnessData.stiffnesses;
    doBalance = false;
    balancerOn = false;
    breakUpOn = false;
  }
}

void GetUpEngine::doHelpMeStuff(GetUpEngineOutput& output)
{
  tooManyTries = true;
  wasHelpMe = true;
  if(theFrameInfo.getTimeSince(breakUpTimestamp) > 100)
  {
    output.stiffnessData.stiffnesses[0] = 0;
    output.stiffnessData.stiffnesses[1] = 0;
  }
  if(std::abs(theInertialData.angle.y()) < 30_deg && std::abs(theInertialData.angle.x()) < 30_deg)
  {
    state = EngineState::finished;
    motionID = GetUpMotion::doNothing;
    checkFinishedState(output);
    lastNotActiveTimestamp = theFrameInfo.time;
  }
  if(theFrameInfo.getTimeSince(helpMeTimestamp) > 5000)
  {
    SystemCall::playSound("helpMe");
    helpMeTimestamp = theFrameInfo.time;
  }
}

void GetUpEngine::setNextJoints(GetUpEngineOutput& output)
{
  if(lineCounter < maxCounter)
  {
    calculateJointDifference(output);
    addJointCompensation(output);
    targetJoints = lineJointRequest;
    MotionUtilities::interpolate(startJoints, targetJoints, ratio, output, theJointAngles);
    lastUnbalanced = output;
    if(balancerOn)
      pidWithCom(output);
  }
}

void GetUpEngine::checkFinishedState(GetUpEngineOutput& output)
{
  //Robot reached a default position
  if(state == EngineState::finishedRecover)
  {
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
    state = EngineState::decideAction;
    checkOwnState(output);
    return;
  }
  //some clipping for the lineCounter, so the condition of the last active keyframe can be checked
  //TODO: the last executed keyframe must not be the last keyframe of the motion because of the conditional keyframe.
  //Does not matter right now, because currently the last keyframe is not a conditional one (for all motions)
  if(lineCounter >= maxCounter)
    lineCounter = maxCounter - 1;
  if(lineCounter == -1)
    lineCounter = 0;
  //Robot could not get up and is now upright again
  if(wasHelpMe)
  {
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
    state = EngineState::working;
    motionID = GetUpMotion::stand;
    tooManyTries = false;
    wasHelpMe = false;
    isContinueTo = false;
    waitForFallenCheck = false;
    errorTriggered = false;
    doFastStand = false;
    initGetUpMotion(output);
    breakUpOn = false;
    wasInWaiting = false;
    isCurrentLineOver(output);
    setNextJoints(output);
  }
  //Get up motion finished and shall still balance to make sure robot will not fall when leaving the getUpEngine
  else if(doBalance && lineCounter < maxCounter && std::abs(theInertialData.angle.y()) < 30_deg)
    doBalanceOut(output);
  //Get up motion finished and GetUpEngine can go inactive
  else if(std::abs(theInertialData.angle.y()) < 30_deg && (mofs[motionID].continueTo == motionID || mofs[motionID].continueTo == GetUpMotion::doNothing))
    shutDownGetUpEngine(output);
  //There is another motion to be executed
  else if(motionID != GetUpMotion::doNothing && mofs[motionID].continueTo != motionID)
  {
    //Is there a wait condition befor executing the next motion?
    if(stepKeyframes || checkConditions(mofs[motionID].lines[lineCounter].waitConditions, true, output))
    {
      state = EngineStates::working;
      doBalance = false;
      motionID = mofs[motionID].continueTo;
      isContinueTo = true;
      initGetUpMotion(output);
      isCurrentLineOver(output);
      setNextJoints(output);
    }
    else
    {
      doBalance = false;
      state = EngineStates::waiting;
      waitTimestamp = theFrameInfo.time;
    }
  }
  //Seems like something went wrong
  else
  {
    doBalance = false;
    tryCounter += 1;
    state = EngineStates::breakUp;
  }
}

void GetUpEngine::setJointStiffness(GetUpEngineOutput& output)
{
  if(lineCounter == 0)
  {
    if(mofs[motionID].baseLimbStiffness.size() > 0)
    {
      for(unsigned int i = 0; i < 2; i++)
        targetJoints.stiffnessData.stiffnesses[i] = mofs[motionID].baseLimbStiffness[0];
      for(unsigned int i = 0; i < 6; i++)
      {
        targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::lShoulderPitch + i)) : Joints::lShoulderPitch + i] = mofs[motionID].baseLimbStiffness[1];
        targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::rShoulderPitch + i)) : Joints::rShoulderPitch + i] = mofs[motionID].baseLimbStiffness[2];
        targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::lHipYawPitch + i)) : Joints::lHipYawPitch + i] = mofs[motionID].baseLimbStiffness[3];
        targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::rHipYawPitch + i)) : Joints::rHipYawPitch + i] = mofs[motionID].baseLimbStiffness[4];
      }
      lineJointRequest.stiffnessData = targetJoints.stiffnessData;
    }
    else
    {
      OUTPUT_ERROR("This Motion does not have baseLimbStiffness!");
      errorTriggered = true;
    }
  }
  else
  {
    for(std::size_t i = 0; i < mofs[motionID].lines[lineCounter].singleMotorStiffnessChange.size(); i++)
      targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].joint) : mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].joint] =
        mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].s;
    lineJointRequest.stiffnessData = targetJoints.stiffnessData;
  }
}

void GetUpEngine::checkOptionalLine(GetUpEngineOutput& output)
{
  if(stepKeyframes)
    return;
  bool doneSkipping = false;

  while(!doneSkipping && lineCounter < maxCounter)
  {
    //We are not in an optionalLine yet but found one
    if(!isInOptionalLine && mofs[motionID].lines[lineCounter].conditions.size() > 0)
      doneSkipping = isInOptionalLine = checkConditions(mofs[motionID].lines[lineCounter].conditions, mofs[motionID].lines[lineCounter].optionalLineAnds, output);
    //We were in an optionalLine and found the end
    else if(isInOptionalLine && !mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine)
    {
      //We found an optionalLine-ElseBlock
      if(mofs[motionID].lines[lineCounter].isElseBlock)
        doneSkipping = false;
      //We found an optionalLine-IfBlock
      else if(mofs[motionID].lines[lineCounter].conditions.size() > 0)
        doneSkipping = isInOptionalLine = checkConditions(mofs[motionID].lines[lineCounter].conditions, mofs[motionID].lines[lineCounter].optionalLineAnds, output);
      //We found the end of the optionalLine without a following else-Block or new optionalLine
      else
        doneSkipping = true;
    }
    //This case should never happen but is a failsave if a case at another place in the getUpEnginge is not correctly implemented
    else
      doneSkipping = isInOptionalLine || !mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine;
    if(!doneSkipping)
    {
      ++lineCounter;
      while(lineCounter < maxCounter && mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine)
        ++lineCounter;
    }
  }
}

template<class C>
bool GetUpEngine::checkConditions(vector<C>& conditions, bool useAnd, GetUpEngineOutput& output)
{
  static_assert(std::is_assignable<Condition, C>::value, "Incompatible types!");

  std::size_t conditionSize = conditions.size();
  if(conditionSize == 0)
    return true;
  //Write the values for the conditions
  float variableValues[numOfConditionVars];
  variableValues[ConditionVar(InertialDataAngleY)] = theInertialData.angle.y().toDegrees();
  variableValues[ConditionVar(InertialDataAngleX)] = theInertialData.angle.x().toDegrees();
  variableValues[ConditionVar(FluctuationY)] = fluctuation.y().toDegrees() * 1.f / Constants::motionCycleTime;
  variableValues[ConditionVar(FluctuationX)] = fluctuation.x().toDegrees() * 1.f / Constants::motionCycleTime;
  float leftArmValue = brokenJointData[2] <= std::fabs(jointDif1Angles.angles[2].toDegrees()) || brokenJointData[3] <= std::fabs(jointDif1Angles.angles[3].toDegrees()) || brokenJointData[4] <= std::fabs(jointDif1Angles.angles[4].toDegrees()) || brokenJointData[5] <= std::fabs(jointDif1Angles.angles[5].toDegrees()) ? 0.f : 1.f;
  float rightArmValue = brokenJointData[8] <= std::fabs(jointDif1Angles.angles[8].toDegrees()) || brokenJointData[9] <= std::fabs(jointDif1Angles.angles[9].toDegrees()) || brokenJointData[10] <= std::fabs(jointDif1Angles.angles[10].toDegrees()) || brokenJointData[11] <= std::fabs(jointDif1Angles.angles[11].toDegrees()) ? 0.f : 1.f;
  variableValues[ConditionVar(BrokenLeftArm)] = isMirror ? rightArmValue : leftArmValue;
  variableValues[ConditionVar(BrokenRightArm)] = isMirror ? leftArmValue : rightArmValue;
  variableValues[ConditionVar(OptionalLineFront)] = theDamageConfigurationBody.optionalLineVersionFront;
  variableValues[ConditionVar(OptionalLineBack)] = theDamageConfigurationBody.optionalLineVersionBack;
  variableValues[ConditionVar(WaitTime)] = static_cast<float>(theFrameInfo.getTimeSince(waitTimestamp));

  //If a waitCondition is checked, this will help to abort the wait time IF it is impossible to fulfill the condition.
  //If a normal condition is checked, this code just wastes processor time and does stuff that has no consequences.
  if(!mofs[motionID].lines[lineCounter].forbidWaitBreak)
  {
    for(std::size_t i = 0; i < conditionSize; i++)
    {
      if(variableValues[conditions[i].variable] <= conditions[i].lowerFloat)
      {
        float dif = variableValues[conditions[i].variable] - conditions[i].lowerFloat;
        if(dif < variableValuesCompare[conditions[i].variable])
          failedWaitCounter += 100.f * Constants::motionCycleTime;
        variableValuesCompare[conditions[i].variable] = dif;
      }
      else if(variableValues[conditions[i].variable] > conditions[i].higherFloat)
      {
        float dif = variableValues[conditions[i].variable] - conditions[i].higherFloat;
        if(dif > variableValuesCompare[conditions[i].variable])
          failedWaitCounter += 100.f * Constants::motionCycleTime;
        variableValuesCompare[conditions[i].variable] = dif;
      }
    }
  }
  for(std::size_t i = 0; i < conditionSize; ++i)
  {
    if(conditions[i].variable == ConditionVar::FluctuationY)
      failedWaitCounter = std::max(failedWaitCounter - 0.7f, 0.f);
    bool check = variableValues[conditions[i].variable] >= conditions[i].lowerFloat && variableValues[conditions[i].variable] <= conditions[i].higherFloat;
    if(conditions[i].isNot)
      check = !check;
    if(!check && useAnd)
      return false;
    else if(check && !useAnd)
      return true;
  }

  return useAnd;
}

template bool GetUpEngine::checkConditions(std::vector<Condition>& conditions, bool useAnd, GetUpEngineOutput& output);
template bool GetUpEngine::checkConditions(std::vector<WaitCondition>& conditions, bool useAnd, GetUpEngineOutput& output);

void GetUpEngine::pidWithCom(GetUpEngineOutput& output)
{
  if(lineCounter > maxCounter || GetUpMotion(motionID) == GetUpMotion::doNothing)
  {
    OUTPUT_ERROR("LineCounter or motionID is wrong in pidWithCom!");
    output.errorTriggered = true;
    return;
  }
  if(theFrameInfo.getTimeSince(lastPIDCom) > 20)
    valPID_I = valPID_D = valPID_P = balanceValue = Vector2f::Zero();
  //Calc base PID values
  lastPIDCom = theFrameInfo.time;
  GetUpPhase::BalanceFactors factors = theGetUpPhase.balanceList[Phase(mofs[motionID].lines[lineCounter].phase)].balanceFactor;
  constexpr float cycletime = Constants::motionCycleTime;
  valPID_D = 0.f * valPID_D + 1.f * ((comDif - oldCom) - goalGrowth);
  valPID_P = valPID_P * 0.f + (comDif - currentGoal) * 1.f;
  valPID_I += comDif - currentGoal;

  //calc values for forward/backward movement
  float nQuadMinY = currentBackwardCOMDif.x() == 0 ? 1.f : 1 / std::pow(currentBackwardCOMDif.x(), factors.powDegreeBack.x());
  float nQuadMaxY = currentForwardCOMDif.x() == 0 ? 1.f : 1 / std::pow(currentForwardCOMDif.x(), factors.powDegreeFront.x());

  float factorY = comDif.x() - currentGoal.x() > 0
                  ? std::min(nQuadMaxY * std::pow(comDif.x() - currentGoal.x(), factors.powDegreeFront.x()), factors.increaseFactorCapFront.x())
                  : std::min(nQuadMinY * std::pow(comDif.x() - currentGoal.x(), factors.powDegreeBack.x()), factors.increaseFactorCapBack.x());

  float pidPY = valPID_P.x() * (factors.borderPIDPX[0] + std::min(factorY, factors.borderPIDPX[1]));
  float pidIY = valPID_I[0] * 0.02f;
  float pidDY = (valPID_D[0] / cycletime) * (factors.borderPIDDX[0] + std::min(factorY, factors.borderPIDDX[1]));

  float balanceValueY = (pidPY + pidIY + pidDY) * cycletime;
  float difYdif = balanceValueY - balanceValue[1];
  balanceValue[1] += difYdif < 0 ? std::max(0.4f * difYdif, -1.f) : std::min(0.4f * difYdif, 1.f); //clip balance value to max 1 degree change per frame

  //calc values for sideways movement
  //TODO side balancing is not really needed and could be removed
  float nQuadMinX = currentBackwardCOMDif.y() == 0 ? 1.f : 1 / std::pow(currentBackwardCOMDif.y(), factors.powDegreeBack.y());
  float nQuadMaxX = currentForwardCOMDif.y() == 0 ? 1.f : 1 / std::pow(currentForwardCOMDif.y(), factors.powDegreeFront.y());

  float factorX = comDif.y() - currentGoal.y() > 0
                  ? std::min(nQuadMaxX * std::pow(comDif.y() - currentGoal.y(), factors.powDegreeFront.y()), factors.increaseFactorCapFront.y())
                  : std::min(nQuadMinX * std::pow(comDif.y() - currentGoal.y(), factors.powDegreeBack.y()), factors.increaseFactorCapBack.y());

  float pidPX = valPID_P.y() * (factors.borderPIDPY[0] + std::min(factorX, factors.borderPIDPY[1]));
  float pidIX = valPID_I[1] * 0.01f;
  float pidDX = (valPID_D[1] / cycletime) * (factors.borderPIDDY[0] + std::min(factorX, factors.borderPIDDY[1]));

  float balanceValueX = (pidPX + pidIX + pidDX) * cycletime;
  float difXdif = balanceValueX - balanceValue[0];
  balanceValue[0] += difXdif < 0 ? std::max(0.2f * difXdif, -2.f) : std::min(0.2f * difXdif, 2.f); //clip balance value to max 2 degree change per frame

  //save D-part value for balanceOut function
  pidDForward = pidDY;

  //apply balance values
  if(!stepKeyframes && balancerOn)
    addBalanceFactor(output, balanceValue[1], balanceValue[0], ratio);

  PLOT("module:GetUpEngine:pid:p", pidPY);
  PLOT("module:GetUpEngine:pid:i", pidIY);
  PLOT("module:GetUpEngine:pid:d", pidDY);
  PLOT("module:GetUpEngine:pid:borderF", currentGoal.x() + currentBackwardCOMDif.x());
  PLOT("module:GetUpEngine:pid:borderB", currentGoal.x() + currentForwardCOMDif.x());
  PLOT("module:GetUpEngine:pid:com", comDif.x());
  PLOT("module:GetUpEngine:pid:balance", balanceValue[1]);
  PLOT("module:GetUpEngine:pid:angleY", theInertialData.angle.y().toDegrees());
  PLOT("module:GetUpEngine:pid:angleX", theInertialData.angle.x().toDegrees());
}

void GetUpEngine::addBalanceFactor(GetUpEngineOutput& output, float factorY, float factorX, float addRatio)
{
  for(JointFactor jointList : mofs[motionID].lines[lineCounter].balanceWithJoints.jointY)
  {
    Joints::Joint joint = jointList.joint;
    float factor = jointList.factor;
    if(isMirror)
      joint = Joints::mirror(joint);
    if((joint == Joints::lAnkleRoll
        || joint == Joints::rAnkleRoll
        || joint == Joints::lHipRoll
        || joint == Joints::rHipRoll) && isMirror)
      factor *= -1.f;
    if(targetJoints.angles[joint] == JointAngles::off || targetJoints.angles[joint] == JointAngles::ignore)
      lineJointRequest.angles[joint] = lastUnbalanced.angles[joint];
    const Rangef qRange(theJointLimits.limits[joint].min, theJointLimits.limits[joint].max);
    output.angles[joint] += convertToAngleSpecialCases(factorY * factor);
    output.angles[joint] = qRange.limit(output.angles[joint]);
  }
  //////////////////
  for(JointFactor jointList : mofs[motionID].lines[lineCounter].balanceWithJoints.jointX)
  {
    Joints::Joint joint = jointList.joint;
    float factor = jointList.factor;
    if(isMirror)
      joint = Joints::mirror(jointList.joint);
    if((joint == Joints::lAnklePitch
        || joint == Joints::rAnklePitch) && isMirror)
      factor *= -1.f;
    if(targetJoints.angles[joint] == JointAngles::off || targetJoints.angles[joint] == JointAngles::ignore)
      lineJointRequest.angles[joint] = lastUnbalanced.angles[joint];
    const Rangef qRange(theJointLimits.limits[joint].min, theJointLimits.limits[joint].max);

    output.angles[joint] += convertToAngleSpecialCases(factorX * factor);
    output.angles[joint] = qRange.limit(output.angles[joint]);
  }
}

void GetUpEngine::addJointCompensation(GetUpEngineOutput& output)
{
  //Reduce jointCompensation of last Keyframe
  //This reduces the joint compensation of the keyframe before, to prevent a overcompensation.
  //Otherwise the compensation of the keyframe before would be reduce not fast enough.
  //Compensation reduction does not work if used with negative values, because every use-case is with positive values.
  JointRequest lastTargetConfig = calculatedCurrentJointRequestWithoutBalance(output); //Joint targets of last Keyframe without balanceing and compensation
  JointRequest startDif = JointRequest();
  for(unsigned int i = 0; i < Joints::numOfJoints; i++)
    startDif.angles[i] = lastTargetConfig.angles[i] - lastLineJointRequestAngles.angles[i]; //Dif of joint targets of last Keyframe with and without balancing and compensation
  if(lineCounter > 0)
  {
    //save last compensation reduce factors
    const auto jointCompensationReducerCopy = jointCompensationReducer;
    if(!mofs[motionID].lines[lineCounter - 1].jointCompensation.empty())
      //jointCompensation index must be 0, because the frameWork assumes, that only(!) the first entry is used.
      for(JointCompensationParams list : mofs[motionID].lines[lineCounter - 1].jointCompensation[0].jointCompensationParams)
      {
        //we compensate the asymetrie in the current motion. no need to handle overcompensation
        if(list.hipPitchDifferenceCompensation)
          continue;

        //If joint started to move again then increase compensationReducer
        if(jointDifPredictedAngles.angles[list.jointDelta] < jointDif1Angles.angles[list.jointDelta]
           && jointDif1Angles.angles[list.jointDelta] - jointDifPredictedAngles.angles[list.jointDelta] > minJointCompensationReduceAngleDif)
          jointCompensationReducer[list.jointDelta] += mofs[motionID].lines[lineCounter].jointCompensation[0].reduceFactorJointCompensation;

        //Update the compensation and remove them from the startJoints.
        for(JointPair jointPair : list.jointPairs)
        {
          Joints::Joint joint = jointPair.joint;
          //We do not need to check the predictJointDif flag, because jointDifPredicted values already do this job
          if(jointDifPredictedAngles.angles[list.jointDelta] < jointDif1Angles.angles[list.jointDelta] //stucked joint moves fast
             && jointDif1Angles.angles[list.jointDelta] - jointDifPredictedAngles.angles[list.jointDelta] > minJointCompensationReduceAngleDif //stuck joint moves fast enough (dif > threshold)
             && lastTargetConfig.angles[joint] != JointAngles::off && lastTargetConfig.angles[joint] != JointAngles::ignore //special case
             && lastLineJointRequestAngles.angles[joint] != JointAngles::off && lastLineJointRequestAngles.angles[joint] != JointAngles::ignore //special case
             && jointCompensationReducer[list.jointDelta] <= 1.f && jointCompensationReducerCopy[list.jointDelta] < jointCompensationReducer[list.jointDelta])// make sure we don't erase more than 100% of the last jointCompensation and only reduce if the joint keeps moving.
            startJoints.angles[joint] += mofs[motionID].lines[lineCounter].jointCompensation[0].reduceFactorJointCompensation * startDif.angles[joint];
        }
      }
  }
  //Compensation reduction end

  //Add jointCompensation for the current keyframe
  lineJointRequest.angles = lineJointRequest2Angles.angles;
  if(!mofs[motionID].lines[lineCounter].jointCompensation.empty())
    for(JointCompensationParams list : mofs[motionID].lines[lineCounter].jointCompensation[0].jointCompensationParams)
    {
      Joints::Joint jointDelta = list.jointDelta;
      if(isMirror)
        jointDelta = Joints::mirror(jointDelta);
      for(JointPair jointPair : list.jointPairs)
      {
        float percent = 0;
        //if minVal is below 0, maxVal is not allowed to be above 0
        //if maxVal is above 0, minVal is not allowed to be below 0
        bool aboveCase = list.minVal >= 0 && list.maxVal >= 0;
        bool belowCase = list.maxVal <= 0 && list.minVal < 0;
        //when hipPitch difference compensation is active, then we can not use the predicted joint angles
        float refAngle = !list.hipPitchDifferenceCompensation ? jointDifPredictedAngles.angles[jointDelta].toDegrees() : Angle(theJointAngles.angles[Joints::lHipPitch] - theJointAngles.angles[Joints::rHipPitch]).toDegrees();
        if(belowCase)
          percent = refAngle < list.maxVal ? std::min((refAngle - list.maxVal) / (list.minVal - list.maxVal), 1.f) : 0.f;
        else if(aboveCase)
          percent = refAngle > list.minVal ? std::min((refAngle - list.minVal) / (list.maxVal - list.minVal), 1.f) : 0.f;
        else
          OUTPUT_ERROR("Error - joint compensation values are wrong: line " << lineCounter << "in motion " << motionID);

        Joints::Joint joint = jointPair.joint;
        //if hipPitch difference compensation is active, then the compensation shall be done different
        if(!list.hipPitchDifferenceCompensation)
        {
          if(isMirror)
            joint = Joints::mirror(joint);
          if((joint == Joints::lAnkleRoll || joint == Joints::rAnkleRoll)
             && isMirror && !list.hipPitchDifferenceCompensation)
            percent *= -1.f;
          //this needs to be done, because we have no prediction for the hipPitch difference
          if(aboveCase)
            lineJointRequest.angles[joint] += Angle::fromDegrees(percent * jointPair.addValue * std::max(refAngle, list.maxVal));
          else
            lineJointRequest.angles[joint] += Angle::fromDegrees(percent * jointPair.addValue * std::min(refAngle, list.minVal));
        }
        else
        {
          if(aboveCase)
            lineJointRequest.angles[joint] += Angle::fromDegrees(percent * jointPair.addValue * std::min(refAngle, list.maxVal));
          else
            lineJointRequest.angles[joint] += Angle::fromDegrees(percent * jointPair.addValue * std::max(refAngle, list.minVal));
        }
      }
    }
}

void GetUpEngine::calculateCOMInSupportPolygon(GetUpEngineOutput& output)
{
  //Offset for the foot area
  std::vector<Pose3f> footOffsets;
  footOffsets.push_back(Pose3f(supportPolygonOffsets.x(), 0.f, 0.f));
  footOffsets.push_back(Pose3f(supportPolygonOffsets.y(), 0.f, 0.f));
  footOffsets.push_back(Pose3f(0.f, -supportPolygonOffsets.z(), 0.f));
  footOffsets.push_back(Pose3f(0.f, supportPolygonOffsets.z(), 0.f));

  //rotation matrix
  Pose3f gyroInTorso(RotationMatrix(Rotation::AngleAxis::unpack(Vector3f(-theInertialData.angle.x(), -theInertialData.angle.y(), 0.f))));

  //get all foot points based on the offsets
  std::vector<Pose3f> limbs;
  for(unsigned i = 0; i < 4; i++)
    limbs.push_back(gyroInTorso.inverse() * theRobotModel.soleLeft * footOffsets[i]);
  for(unsigned i = 0; i < 4; i++)
    limbs.push_back(gyroInTorso.inverse() * theRobotModel.soleRight * footOffsets[i]);

  //sort them by z coordinate
  std::sort(limbs.begin(), limbs.end(),
            [](const auto & l1, const auto & l2)
  {
    return l1.translation.z() < l2.translation.z();
  });
  float groundHeight = limbs[0].translation.z();
  std::vector<Pose3f> supportPolygon;

  //sort all foot point out, that are to far above the ground. these points have no ground contact.
  for(Pose3f& p : limbs)
    if(p.translation.z() < groundHeight + 100.f)
      supportPolygon.push_back(p);
  Pose3f supportCenter(0.f, 0.f, 0.f);

  //At least 2 points are needed for an area
  if(supportPolygon.size() < 2)
    return;

  //calculate middle point of the support polygon
  //sort by y
  std::sort(supportPolygon.begin(), supportPolygon.end(),
            [](const Pose3f& p1, const Pose3f& p2) -> bool
  {
    return p1.translation.y() < p2.translation.y();
  });

  float minY = supportPolygon[0].translation.y();
  float maxY = supportPolygon[supportPolygon.size() - 1].translation.y();

  //sort by x
  std::sort(supportPolygon.begin(), supportPolygon.end(),
            [](const Pose3f& p1, const Pose3f& p2) -> bool
  {
    return p1.translation.x() < p2.translation.x();
  });

  float minX = supportPolygon[0].translation.x();
  float maxX = supportPolygon[supportPolygon.size() - 1].translation.x();

  //sort by z
  std::sort(supportPolygon.begin(), supportPolygon.end(),
            [](const Pose3f& p1, const Pose3f& p2) -> bool
  {
    return p1.translation.z() < p2.translation.z();
  });

  float z = supportPolygon[0].translation.z();

  //save all 4 point of the foot area
  std::vector<Pose3f> supportPolygonCopy;
  Pose3f a = Pose3f(minX, minY, z);
  Pose3f b = Pose3f(maxX, minY, z);
  Pose3f c = Pose3f(minX, maxY, z);
  Pose3f d = Pose3f(maxX, maxY, z);
  supportPolygonCopy.push_back(a);
  supportPolygonCopy.push_back(b);
  supportPolygonCopy.push_back(d);
  supportPolygonCopy.push_back(c);
  output.supportPolygon = supportPolygonCopy; //foot area

  //calculate middle point of foot area
  supportCenter += a.translation.array();
  supportCenter += b.translation.array();
  supportCenter += c.translation.array();
  supportCenter += d.translation.array();
  supportCenter.translation.array() /= 4;

  output.gyroInTorso = gyroInTorso;
  output.supportCenterInTorso = gyroInTorso * supportCenter; //middlepoint in torso
  output.comInTorso = theRobotModel.centerOfMass; //com
  Pose3f comInGyro = gyroInTorso.inverse() * output.comInTorso; //com in gyro
  Vector3f balanceDif = comInGyro.translation - supportCenter.translation;//difference of com in torso and middle point in foot area. z-axis is ignored

  oldCom2 = oldCom;
  oldCom = comDif;
  comDif = { balanceDif.x(), balanceDif.y() };
}

JointRequest GetUpEngine::calculatedCurrentJointRequestWithoutBalance(GetUpEngineOutput& output)
{
  if(lineCounter > 0)
  {
    JointRequest lastTarget = JointRequest();
    lastTarget.angles[Joints::headYaw] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter - 1].head[0]);
    lastTarget.angles[Joints::headPitch] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter - 1].head[1]);
    for(unsigned int i = 0; i < 6; i++)
    {
      lastTarget.angles[Joints::lShoulderPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter - 1].leftArm[i]);
      lastTarget.angles[Joints::rShoulderPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter - 1].rightArm[i]);
      lastTarget.angles[Joints::lHipYawPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter - 1].leftLeg[i]);
      lastTarget.angles[Joints::rHipYawPitch + i] = convertToAngleSpecialCases(mofs[motionID].lines[lineCounter - 1].rightLeg[i]);
    }
    if(isMirror)
    {
      JointRequest mirroredJoints;
      mirroredJoints.mirror(lastTarget);
      lastTarget.angles = mirroredJoints.angles;
    }
    return lastTarget;
  }
  return JointRequest();
}

void GetUpEngine::updateSensorArray(GetUpEngineOutput& output)
{
  lastTorsoAngle.push_front(theInertialData.angle.head<2>());

  Vector2a vel = lastTorsoAngle[0] - lastTorsoAngle[1];
  Vector2a acc = vel - (lastTorsoAngle[1] - lastTorsoAngle[2]);

  fluctuation = Vector2a(std::abs(acc.x()), std::abs(acc.y())) + Vector2a(std::abs(vel.x()), std::abs(vel.y()));

  if(lineCounter < maxCounter && state > EngineState::decideAction && maxCounter > 0)
    updateLineValues(output);
  calculateCOMInSupportPolygon(output);
}

void GetUpEngine::doBalanceOut(GetUpEngineOutput& output)
{
  if(doBalance)
  {
    state = EngineState::balanceOut;
    doBalance = false;
    balancerOn = true;
    goalGrowth = Vector2f::Zero();
    doBalanceOutTimestamp = theFrameInfo.time;
  }
  ratio = 1.f;
  if(!isInBreakUpRange(output) && !output.isLeavingPossible)
  {
    if(theFrameInfo.getTimeSince(doBalanceOutTimestamp) > balanceOutParams.maxTime
       || (std::abs(fluctuation.y()) < balanceOutParams.minFluctuation
           && theInertialData.angle.y() < balanceOutParams.minForwardAngle
           && theInertialData.angle.y() > balanceOutParams.minBackwardAngle
           && std::fabs(pidDForward) < balanceOutParams.minPIDDValue))
    {
      if(mofs[motionID].continueTo == motionID)
        output.isLeavingPossible = true;
      else
        state = EngineState::finished;
    }
    if(state != EngineState::finished && state != EngineState::breakUp)
      setNextJoints(output);
    else if(state == EngineState::finished)
      checkFinishedState(output);
  }
}

void GetUpEngine::shutDownGetUpEngine(GetUpEngineOutput& output)
{
  jointsBalanceY = std::vector<Joints::Joint>();
  jointsBalanceX = std::vector<Joints::Joint>();
  wasInWaiting = false;
  doBalance = false;
  tooManyTries = false;
  wasHelpMe = false;
  doFastStand = false;
  getUpFinishedTimStamp = theFrameInfo.time;
  output.isLeavingPossible = true;
  state = EngineStates::off;
  motionID = GetUpMotion::doNothing;
  ratio = 1.f;
  framesTillEnd = 1.f;
  tryCounter = 0;
  retryMotionCounter = 0;
  fastRecover = false;
  specialActionCheck = false;
  armLeftCheck = false;
  armRightCheck = false;
  sideCheck = false;
  waitForFallenCheck = false;
  errorTriggered = false;
  isContinueTo = false;
}

Angle GetUpEngine::convertToAngleSpecialCases(float angle)
{
  return angle == JointAngles::off || angle == JointAngles::ignore ? static_cast<Angle>(angle) : Angle::fromDegrees(angle);
}
MAKE_MODULE(GetUpEngine, motionControl)
