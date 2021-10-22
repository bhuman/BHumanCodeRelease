/*
 * @file KeyframeMotionEngine.cpp
 * @author <A href="mailto:s_ksfo6n@uni-bremen.de">Philip Reichenberg</A>
 */

#include "KeyframeMotionEngine.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Debugging/DebugDrawings3D.h"

using namespace std;

KeyframePhase::KeyframePhase(KeyframeMotionEngine& engine, const MotionRequest& motionRequest, const MotionPhase& lastPhase) :
  MotionPhase(motionRequest.motion == MotionRequest::getUp ? MotionPhase::getUp : MotionPhase::keyframeMotion),
  engine(engine)
{
  currentKeyframeMotionRequest = motionRequest;
  if(lastPhase.type == MotionPhase::getUp || lastPhase.type == MotionPhase::keyframeMotion)
  {
    const auto& lastKeyframePhase = static_cast<const KeyframePhase&>(lastPhase);
    lastKeyframeMotionRequest = lastKeyframePhase.currentKeyframeMotionRequest;
    lastJointRequest = lastKeyframePhase.jointRequestOutput;
    lastUnbalanced = lastKeyframePhase.lastUnbalanced;
    jointRequestOutput = lastKeyframePhase.jointRequestOutput;
    // special case for demo wave
    if(lastKeyframePhase.motionID != KeyframeMotionID::demoBannerWave && lastKeyframePhase.motionID != KeyframeMotionID::demoBannerWaveInitial)
    {
      FOREACH_ENUM(Joints::Joint, joint)
      {
        lastJointRequest.angles[joint] = lastKeyframePhase.preHeatAdjustment.angles[joint];
        jointRequestOutput.angles[joint] = lastKeyframePhase.preHeatAdjustment.angles[joint];
      }
      engine.theEnergySaving.shutDown();
    }
  }
  else
  {
    lastJointRequest = engine.theJointRequest;
    jointRequestOutput = engine.theJointRequest;
    lastUnbalanced = engine.theJointRequest;
  }
  if(lastPhase.type != MotionPhase::keyframeMotion)
    engine.theEnergySaving.shutDown();
  state = EngineState::off;
  motionID = KeyframeMotionID::stand;
  lineStartTimestamp = 0;
  lastNotActiveTimestamp = 0;
  waitTimestamp = 0;
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
  frontHackTriggered = false;

  jointDiff1Angles.angles = JointRequest().angles;

  jointDiffPredictedAngles.angles = JointRequest().angles;
  lastLineJointRequestAngles.angles = JointRequest().angles;
  jointFailureRequestAngles.angles = JointRequest().angles;
  jointFailureRequestCurrentBordersAngles.angles = JointRequest().angles;
  jointRequestWithoutFailureBehaviorAngles.angles = JointRequest().angles;

  jointsBalanceY = jointsBalanceX = std::vector<Joints::Joint>();

  lastBackwardAngleBreakUp = lastForwardAngleBreakUp = currentBackwardAngleBreakUp = currentForwardAngleBreakUp = Vector2a::Zero();
  lastBackwardCOMDiff = lastForwardCOMDiff = currentBackwardCOMDiff = currentForwardCOMDiff = Vector2f::Zero();

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
  nonGetUpMotionCheck = false;
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
  balancerOn = false;
  isFirstLine = false;
  breakUpOn = false;
  isMirror = false;
  isInOptionalLine = false;
  wasInWaiting = false;
  getUpMirror = false;
  didFirstGetUp = false;
  finishGettingUp = false;
  doSlowKeyframeAfterFall = false;

  //init ringbuffer
  lastTorsoAngle.push_front(Vector2a::Zero());
  lastTorsoAngle.push_front(Vector2a::Zero());
  lastTorsoAngle.push_front(Vector2a::Zero());
}

void KeyframeMotionEngine::update(KeyframeMotionGenerator& output)
{
#ifndef NDEBUG
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lAnklePitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rAnklePitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lKneePitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rKneePitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lHipPitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rHipPitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lHipYawPitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lShoulderPitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rShoulderPitch");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lAnkleRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rAnkleRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lShoulderRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rShoulderRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lElbowRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rElbowRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lElbowYaw");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rElbowYaw");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lWristYaw");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rWristYaw");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:lHipRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:jointDiff1:rHipRoll");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:p");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:i");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:d");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:balance");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:borderF");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:borderB");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:com");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:angleY");
  DECLARE_PLOT("module:KeyframeMotionEngine:pid:angleX");
  DECLARE_PLOT("module:KeyframeMotionEngine:ratio");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:getMotionLine");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:getCurrentAnglesInMof");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:calibrate");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:addMotionFailure");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:clearJointFailure");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:toggleBalancer");
  DECLARE_DEBUG_DRAWING3D("module:KeyframeMotionEngineOutput:coordinates", "robot");

  KeyframeMotionID motionIDInfo = KeyframeMotionID::decideAutomatic;
  int motionLine = 0;
  MODIFY("module:KeyframeMotionEngine:getMotionLineID", motionIDInfo);
  MODIFY("module:KeyframeMotionEngine:getMotionLineNumber", motionLine);
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:getMotionLine")
    if(motionIDInfo != KeyframeMotionID::decideAutomatic)
      getMotionLine(motionIDInfo, motionLine);
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:getCurrentAnglesInMof")
    getCurrentAnglesInMof();

  MODIFY_ONCE("module:KeyframeMotionEngine:setSteppingKeyframes", stepKeyframes);
  MODIFY("module:KeyframeMotionEngine:calculateDrawing", calculateDrawing);
  MODIFY("module:KeyframeMotionEngine:startCalibration", setCalibrationState);
  MODIFY("module:KeyframeMotionEngine:setCalibrationPhase", calibrationPhase);

  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:calibrate")
  {
    isCalibrationState = true;
    initPhase = true;
    isCalibrationState = true;
  }

  MODIFY("module:KeyframeMotionEngine:setMotionIDFailure", motionIDFailure);
  MODIFY("module:KeyframeMotionEngine:setLineCounterFailureStart", lineCounterFailureStart);
  MODIFY("module:KeyframeMotionEngine:setLineCounterFailureEnd", lineCounterFailureEnd);
  MODIFY("module:KeyframeMotionEngine:setRatioFailureStart", ratioFailureStart);
  MODIFY("module:KeyframeMotionEngine:setRatioFailureEnd", ratioFailureEnd);
  MODIFY("module:KeyframeMotionEngine:setJointFailure", joint);
  editMode(output);
#endif

  output.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    return std::make_unique<KeyframePhase>(*this, motionRequest, lastPhase);
  };

  output.setPhaseInformation = [&output](const int line, const float ratio, const KeyframeMotionRequest::KeyframeMotionID motion)
  {
    output.line = line;
    output.ratio = ratio;
    output.motion = motion;
  };
}

void KeyframeMotionEngine::update(GetUpGenerator& output)
{
  output.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    return std::make_unique<KeyframePhase>(*this, motionRequest, lastPhase);
  };
}

void KeyframeMotionEngine::getMotionLine(KeyframeMotionID motionID, int motionLine)
{
  std::string returnString = "";
  MofLine lines = mofs[motionID].lines[motionLine];
  std::string head;
  std::string lArm;
  std::string rArm;
  std::string lLeg;
  std::string rLeg;

  head += std::to_string(lines.head[0]);
  head += " ";
  head += std::to_string(lines.head[1]);
  head += " ";
  for(unsigned int i = 0; i < 6; i++)
  {
    lArm += std::to_string(lines.leftArm[i]);
    lArm += " ";
    rArm += std::to_string(lines.rightArm[i]);
    rArm += " ";
    lLeg += std::to_string(lines.leftLeg[i]);
    lLeg += " ";
    rLeg += std::to_string(lines.rightLeg[i]);
    rLeg += " ";
  }
  returnString += head + lArm + rArm + lLeg + rLeg;
  returnString += "1 2000";
  OUTPUT_TEXT(returnString);
}

void KeyframeMotionEngine::getCurrentAnglesInMof()
{
  std::string returnString;
  std::string head = "head = [ \n";
  std::string lArm = "leftArm = [ \n";
  std::string rArm = "rightArm = [ \n";
  std::string lLeg = "leftLeg = [ \n";
  std::string rLeg = "rightLeg = [ \n";
  head += std::to_string(static_cast<int>(std::round(theJointRequest.angles[0].toDegrees()))) + ", \n";
  head += std::to_string(static_cast<int>(std::round(theJointRequest.angles[1].toDegrees()))) + ", \n ]; \n";
  for(int i = 0; i < 6; i++)
  {
    lArm += std::to_string(static_cast<int>(std::round(theJointRequest.angles[Joints::lShoulderPitch + i].toDegrees()))) + ", \n";
    rArm += std::to_string(static_cast<int>(std::round(theJointRequest.angles[Joints::rShoulderPitch + i].toDegrees()))) + ", \n";
    lLeg += std::to_string(static_cast<int>(std::round(theJointRequest.angles[Joints::lHipYawPitch + i].toDegrees()))) + ", \n";
    rLeg += std::to_string(static_cast<int>(std::round(theJointRequest.angles[Joints::rHipYawPitch + i].toDegrees()))) + ", \n";
  }

  lArm += " ]; \n";
  rArm += " ]; \n";
  lLeg += " ]; \n";
  rLeg += " ]; \n";
  returnString += head + lArm + rArm + lLeg + rLeg;
  OUTPUT_TEXT(returnString);
}

bool KeyframePhase::isDone(const MotionRequest& motionRequest) const
{
  const bool sitDownWaitForRequest = currentKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion != KeyframeMotionID::sitDown || !engine.theGroundContactState.contact || motionRequest.motion != MotionRequest::playDead;
  const bool isSitDownBreakUp = currentKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionID::sitDown && state == EngineState::breakUp;
  return (((state == EngineState::waitForRequest && sitDownWaitForRequest) || isSitDownBreakUp) // (Motion is done OR sitDown special) cases AND request changed
          && (currentKeyframeMotionRequest.motion != motionRequest.motion || currentKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion != motionRequest.keyframeMotionRequest.keyframeMotion))
         || (state == EngineState::balanceOut && motionRequest.isWalking() && type == MotionPhase::getUp && engine.theGroundContactState.contact); // Early switch to walkPhase
}

void KeyframePhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  lastJointRequest = jointRequestOutput;
  ASSERT(jointRequestOutput.isValid());
  jointRequest = jointRequestOutput;
  odometryOffset = odometry;
  motionInfo.isMotionStable = engine.mofs[motionID].isMotionStable;
  motionInfo.executedKeyframeMotion.keyframeMotion = motionID;
  motionInfo.executedKeyframeMotion.mirror = isMirror;
  if(type == MotionPhase::getUp)
    motionInfo.getUpTryCounter = tryCounter;
}

void KeyframePhase::update()
{
#ifndef NDEBUG
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:toggleBalancer")
    balancerOn = !balancerOn;
#endif
  odometry = Pose2f();

  //If the KeyframeMotionEngine went active AND the first keyframe needs balancing, then we would need to wait 3 motion frames to make sure that the last 3 calculated "com" values
  //(comDif, oldCom, oldCom2) are from the current get up try. But because there is always at least 1 keyframe before a keyframe that needs balancing, this "problem" will never happen.
  updateSensorArray();
  jointFailureRequestAngles.angles = preHeatAdjustment.angles;
  jointRequestOutput.angles = jointRequestWithoutFailureBehaviorAngles.angles;
  //check for special events
  doManipulateOwnState();

  //are we in a calibration state?
  if(state == EngineState::calibrationState)
    doCalibrationState();
  //is this the first frame the KeyframeMotionEngine is on?
  if(state == EngineState::off)
  {
    //If there is a problem with the sensory update, wait until it is fixed (V6 head flex problem)
    if(engine.theGyroOffset.bodyDisconnect)
      return;
    retryMotionCounter = 0;
    //Init pastJointAngles. This is done to prevent wrong calculated joint differences in calculateJointDifference
    for(std::size_t i = 0; i < pastJointAnglesAngles.capacity(); i++)
    {
      JointRequestAngles lastRequest;
      lastRequest.angles = engine.theJointAngles.angles;
      pastJointAnglesAngles.push_front(lastRequest);
    }
    //Init jointDif. This is done to prevent wrong calculated joint differences in calculateJointDifference
    for(std::size_t i = 0; i < jointDiff1Angles.angles.size(); i++)
    {
      jointDiff1Angles.angles[i] = 0.f;
      jointDiffPredictedAngles.angles[i] = 0.f;
      lastLineJointRequestAngles.angles[i] = JointAngles::off;
    }
    lastNotActiveTimestamp = engine.theFrameInfo.time;
    waitForFallenCheck = true;
    state = EngineState::decideAction;
  }
  //Wait until we can start a new get up try.
  if(state == EngineState::breakUp)
    waitForFallen();
  //Wait for a requested motion
  if(state == EngineState::waitForRequest)
  {
    // Get up finished but robot fell down again while waiting for a motion request change -> isInBreakUpRange changes the state and isDone() is false again
    static_cast<void>(isInBreakUpRange());
    // no return, to allow energy saving at the end
  }
  //Decide what to do
  if(state == EngineState::decideAction)
  {
    //Check which motion shall be executed
    checkMotion();
    int counter = 0;
    //This is done because if the set motion has no lines (which should only happen due to a human error), we want to skip this motion
    while(maxCounter == 0 && counter < 8)
    {
      state = EngineState::decideAction;
      checkMotion();
      counter++;
      errorTriggered = false;
    }
    //We found no valid get up motion.
    if(counter == 8)
      state = EngineState::helpMeState;
  }
  //Check if we waited long enough and can continue our motion.
  //TODO not sure if wait time currently work with non get up motions (they are also not used by them yet)
  if(state == EngineState::waiting)
  {
    int waitTime = engine.mofs[motionID].lines[lineCounter].waitConditions.size() == 1 ? engine.mofs[motionID].lines[lineCounter].waitConditions[0].maxWaitTime : 1000;
    //Waiting over?
    if(checkConditions(engine.mofs[motionID].lines[lineCounter].waitConditions, true) || engine.theFrameInfo.getTimeSince(waitTimestamp) >= waitTime || failedWaitCounter > 8.f)
    {
      wasInWaiting = true;
      state = currentKeyframeMotionRequest.motion == MotionRequest::getUp ? EngineState::working : EngineState::workingAction;
      initCurrentLine();
      updateLineValues();
    }
    //We are in break up range and need to break the motion?
    else if(!isInBreakUpRange())
    {
      //We are still in a waiting time. We set the next joints so we can still balance while we wait.
      ratio = 1.f;
      setNextJoints();
    }
  }
  //Check retry state. This is an optimization so we can retry motions without using the tryCounter.
  if(state == EngineState::retryState)
  {
    //Wait time over?
    if(engine.theFrameInfo.getTimeSince(retryTimestamp) > retryTime)
    {
      //Init all the motion stuff so we can retry the motion from a given line.
      state = EngineState::working;
      if(lineCounter == 0)
        initGetUpMotion();
      else
      {
        initBalancerValues(true);
        initCurrentLine();
        ratio = 0.f;
        updateLineValues();
      }
      duration = 1000;
    }
  }
  //We are at the end of the get up motion and are standing. Keep balancing.
  if(state == EngineState::balanceOut)
    doBalanceOut();
  //Check if we can continue the current motion and set the next joints.
  if(state == EngineState::recoverFallen || state == EngineState::working || state == EngineState::workingAction)
  {
    //calc current ratio;
    isCurrentLineOver();
    //Need to break the motion?
    isInBreakUpRange();
    //continue motion
    if(state == EngineState::working || state == EngineState::recoverFallen || state == EngineState::workingAction)
      setNextJoints();
  }
  if(state == EngineState::finished || state == EngineState::finishedRecover)
    checkFinishedState();
  //We have no get up trys left.
  if(state == EngineState::helpMeState)
    doHelpMeStuff();
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(jointRequestOutput.stiffnessData.stiffnesses[joint] == 0)
      jointRequestOutput.angles[joint] = JointAngles::off;
  }
  checkJointFailureBehavior();

  // apply heat adjustment
  preHeatAdjustment.angles = jointRequestOutput.angles;
  if(engine.mofs[motionID].energySavingLegs
     && ((motionID != KeyframeMotionID::demoBannerWave && motionID != KeyframeMotionID::demoBannerWaveInitial && state == EngineState::waitForRequest)
         || motionID != KeyframeMotionID::demoBannerWave || motionID != KeyframeMotionID::demoBannerWaveInitial))
    engine.theEnergySaving.applyHeatAdjustment(jointRequestOutput, engine.mofs[motionID].energySavingLegs, engine.mofs[motionID].energySavingArms, false, false);

  engine.theKeyframeMotionGenerator.setPhaseInformation(lineCounter, ratio, motionID);
}

//The pastJointAngles values need to reset for every motion try
//also it needs to wait pastJointAngles.size() frames until it can start the checks. Otherwise the joint angle differences will be large, because
//the sensors are really slow
void KeyframePhase::calculateJointDifference()
{
  JointRequestAngles lastRequest;
  for(std::size_t j = 0; j < pastJointAnglesAngles.back().angles.size(); j++)
    lastRequest.angles[j] = jointRequestOutput.angles[j] != JointAngles::ignore && jointRequestOutput.angles[j] != JointAngles::off && targetJoints.stiffnessData.stiffnesses[j] != 0
                            ? jointRequestOutput.angles[j]
                            : engine.theJointAngles.angles[j];

  pastJointAnglesAngles.push_front(lastRequest);
  //Every Joint, that has a joint compensation shall be checked if the jointDif shall be predicted 3 frames into the future.
  std::vector<bool> predict = std::vector<bool>(Joints::numOfJoints);
  if(!engine.mofs[motionID].lines[lineCounter].jointCompensation.empty())
    for(JointCompensationParams list : engine.mofs[motionID].lines[lineCounter].jointCompensation[0].jointCompensationParams)
      predict[list.jointDelta] = list.predictJointDif;

  //Calculated current joint difference of "set" angles and "reached" angles.
  for(std::size_t i = 0; i < pastJointAnglesAngles.back().angles.size(); i++)
  {
    Angle diff = pastJointAnglesAngles.back().angles[i] - engine.theJointAngles.angles[i];
    Angle prevDiff = jointDiff1Angles.angles[i];
    jointDiff1Angles.angles[i] = pastJointAnglesAngles.back().angles[i] != -2000_deg
                                 ? 0.6f * prevDiff + 0.4f * diff
                                 : 0.f;
    jointDiffPredictedAngles.angles[i] = jointDiff1Angles.angles[i];

    if(predict[i])
    {
      // predict 3 frames into the future
      jointDiffPredictedAngles.angles[i] += 3.f * (jointDiff1Angles.angles[i] - prevDiff);
      // don't overpredict
      jointDiffPredictedAngles.angles[i] = std::abs(jointDiff1Angles.angles[i]) < std::abs(jointDiffPredictedAngles.angles[i]) ? jointDiff1Angles.angles[i] : jointDiffPredictedAngles.angles[i];
    }
  }
  //in addJointCompensation the joints are mirrored when the motion is mirrored. Thats why we make sure rHYP and lHYP are the same
  jointDiff1Angles.angles[Joints::rHipYawPitch] = jointDiff1Angles.angles[Joints::lHipYawPitch];
  jointDiffPredictedAngles.angles[Joints::rHipYawPitch] = jointDiffPredictedAngles.angles[Joints::lHipYawPitch];

  PLOT("module:KeyframeMotionEngine:jointDiff1:lAnklePitch", jointDiffPredictedAngles.angles[Joints::lAnklePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rAnklePitch", jointDiffPredictedAngles.angles[Joints::rAnklePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lKneePitch", jointDiffPredictedAngles.angles[Joints::lKneePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rKneePitch", jointDiffPredictedAngles.angles[Joints::rKneePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lHipPitch", jointDiffPredictedAngles.angles[Joints::lHipPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rHipPitch", jointDiffPredictedAngles.angles[Joints::rHipPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lHipYawPitch", jointDiffPredictedAngles.angles[Joints::lHipYawPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lShoulderPitch", jointDiffPredictedAngles.angles[Joints::lShoulderPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rShoulderPitch", jointDiffPredictedAngles.angles[Joints::rShoulderPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lAnkleRoll", jointDiffPredictedAngles.angles[Joints::lAnkleRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rAnkleRoll", jointDiffPredictedAngles.angles[Joints::rAnkleRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lShoulderRoll", jointDiffPredictedAngles.angles[Joints::lShoulderRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rShoulderRoll", jointDiffPredictedAngles.angles[Joints::rShoulderRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lElbowRoll", jointDiffPredictedAngles.angles[Joints::lElbowRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rElbowRoll", jointDiffPredictedAngles.angles[Joints::rElbowRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lElbowYaw", jointDiffPredictedAngles.angles[Joints::lElbowYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rElbowYaw", jointDiffPredictedAngles.angles[Joints::rElbowYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lWristYaw", jointDiffPredictedAngles.angles[Joints::lWristYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rWristYaw", jointDiffPredictedAngles.angles[Joints::rWristYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lHipRoll", jointDiffPredictedAngles.angles[Joints::lHipRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rHipRoll", jointDiffPredictedAngles.angles[Joints::rHipRoll].toDegrees());
}

void KeyframePhase::checkMotion()
{
  isMirror = false;
  const Angle& bodyAngleY = engine.theInertialData.angle.y();
  const Angle& bodyAngleX = engine.theInertialData.angle.x();
  //This is for getting up
  if(currentKeyframeMotionRequest.motion == MotionRequest::getUp || finishGettingUp)
  {
    finishGettingUp = true;
    //First check if robot did a non get up motion before
    if(!nonGetUpMotionCheck && std::abs(bodyAngleY) < 30_deg && lastKeyframeMotionRequest.motion >= MotionRequest::keyframeMotion)
    {
      //TODO lastKeyframeMotionRequest must be set from previous keyFramePhase
      nonGetUpMotionCheck = true;
      //SumoPosition, aka Genuflect for field players
      if(lastKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStand)
      {
        state = EngineState::working;
        motionID = KeyframeMotionID::recoverFromSumo;
      }
      //SumoPositionDefender, aka Genuflect for Keeper
      else if(lastKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStandDefender)
      {
        state = EngineState::working;
        motionID = KeyframeMotionID::recoverFromGenu;
      }
      //Robot is sitting
      else if(lastKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::sitDownKeeper
              || lastKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::sitDown)
      {
        state = EngineState::working;
        motionID = KeyframeMotionID::stand;
      }
    }
    // Either get up went falsly active and robot is still standig upright,
    // or while getting up (and doing a recover motion) a human is holding him upright
    // -> go directly into standing
    if(state == EngineState::decideAction
       && std::abs(bodyAngleY) < 20_deg
       && std::abs(bodyAngleX) < 25_deg
       && engine.theJointAngles.angles[Joints::lKneePitch] < 60_deg
       && engine.theJointAngles.angles[Joints::rKneePitch] < 60_deg
       && engine.theRobotModel.soleLeft.translation.z() < -200.f
       && engine.theRobotModel.soleRight.translation.z() < -200.f)
    {
      state = EngineState::working;
      motionID = KeyframeMotionID::stand;
      doFastStand = true;
    }
    //No get up allowed
    if(engine.theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::allBroken)
    {
      state = EngineState::helpMeState;
      motionID = KeyframeMotionID::decideAutomatic;
    }
    //Is Robot in a splitting or sitting position?
    if(state == EngineState::decideAction && std::abs(bodyAngleX) < 35_deg && std::abs(bodyAngleY) < 35_deg)
    {
      state = EngineState::working;
      motionID = KeyframeMotionID::sit;
    }
    //Is Robot lying on side?
    if(!sideCheck && state == EngineState::decideAction)
    {
      sideCheck = true;
      //NAO lies on his right arm
      if(std::abs(bodyAngleX) > 40_deg)
      {
        state = EngineState::recoverFallen;
        if(bodyAngleX < -40_deg)
          isMirror = true;
        //recover to back?
        if(bodyAngleY < -10_deg)
          motionID = KeyframeMotionID::recoverFromSideBack;
        //recover to front!
        else
          motionID = KeyframeMotionID::recoverFromSideFront;
      }
    }
    //Robot is lying on ground. Do a fast recover to see if all problem can be solved by that
    if(!fastRecover && state == EngineState::decideAction)
    {
      fastRecover = true;
      state = EngineState::recoverFallen;
      motionID = KeyframeMotionID::recoverFast;
    }
    //Is Robot lying on his arms?
    if((!armLeftCheck || !armRightCheck) && state == EngineState::decideAction)
    {
      armLeftCheck = true;
      armRightCheck = true;
      //lying on front
      if(bodyAngleY > 45_deg)
      {
        //check both arms
        if((engine.theRobotModel.limbs[Limbs::wristLeft].translation.y() < 90.f && engine.theJointAngles.angles[Joints::lShoulderPitch] > 0_deg)
           || (engine.theRobotModel.limbs[Limbs::wristRight].translation.y() > -90.f && engine.theJointAngles.angles[Joints::rShoulderPitch] > 0_deg))
        {
          isMirror = false;
          state = EngineState::recoverFallen;
          motionID = KeyframeMotionID::recoverArmLeftFrontLyingOn;
        }
        if(KeyframeMotionID(motionID) == KeyframeMotionID::decideAutomatic)
          state = EngineState::decideAction;
      }
    }
    //Robot can start a get up motion.
    if(state == EngineState::decideAction)
    {
      //Robot is upright enough to go into sit (which continues to stand)
      if(std::abs(bodyAngleY) < 30_deg && std::abs(bodyAngleX) < 20_deg)
      {
        state = EngineState::working;
        motionID = KeyframeMotionID::sit;
      }
      //Start normal get up for front/back
      else
      {
        state = EngineState::working;
        tryCounter >= engine.maxTryCounter ? tooManyTries = true : motionID = (bodyAngleY > 0_deg ? KeyframeMotionID::front : KeyframeMotionID::back);
      }
    }
    if(state == EngineState::working || state == EngineState::recoverFallen)
      initGetUpMotion();
    else if(state == EngineState::helpMeState)
      doHelpMeStuff();
    //Should never trigger, but better save than sorry.
    else if(state == EngineState::decideAction)
    {
      SystemCall::say("This Code should not be executed");
      ANNOTATION("KeyframeMotionEngine", "This Code should not be executed");
      for(std::size_t i = 0; i < jointRequestOutput.angles.size(); i++)
        jointRequestOutput.angles[i] = JointAngles::ignore;
    }
  }
  else
  {
    motionID = currentKeyframeMotionRequest.keyframeMotionRequest.keyframeMotion;
    state = EngineState::workingAction;
    isMirror = currentKeyframeMotionRequest.keyframeMotionRequest.mirror;
    initGetUpMotion();
  }
}

bool KeyframePhase::isInBreakUpRange()
{
  if(engine.stepKeyframes)
    return false;
  const Angle yTorso = engine.theInertialData.angle.y();
  const Angle xTorso = engine.theInertialData.angle.x();

  //Need to break?
  const bool retryFront = checkRetryForFront();
  if(((engine.theFilteredCurrent.legMotorMalfunction && engine.motorMalfunctionBreakUp)  //motor malfunction
      || retryFront //retry front get up
      || ((yTorso > currentForwardAngleBreakUp.x() //tilting forward
           || yTorso < currentBackwardAngleBreakUp.x() //tilting backward
           || xTorso > currentForwardAngleBreakUp.y() //tilting left
           || xTorso < currentBackwardAngleBreakUp.y()) //tilting right
          && ((engine.theGroundContactState.contact && motionID >= KeyframeMotionID::firstNonGetUpAction) //ground contact while executing non get up motion
              || motionID < KeyframeMotionID::firstNonGetUpAction))) //ignore ground contact when getting up
     && breakUpOn)
  {
    if(!checkMotionSpecificActions())
    {
      //Can we do a get up try afterwards?
      if(tryCounter >= engine.maxTryCounter && motionID < KeyframeMotionID::firstNonGetUpAction)
        state = EngineState::helpMeState;
      else
      {
        retryMotionCounter += retryFront ? 1 : 0;
        if(SystemCall::getMode() != SystemCall::simulatedRobot && !retryFront)
          tryCounter += 1;
        state = EngineState::breakUp;
        waitForFallenCheck = true;
        if(SystemCall::getMode() != SystemCall::logFileReplay)
          SystemCall::say("Abort");
        for(std::size_t i = 0; i < jointRequestOutput.angles.size(); i++)
          jointRequestOutput.stiffnessData.stiffnesses[i] = 10;
      }
      return true;
    }
  }
  return false;
}

bool KeyframePhase::checkMotionSpecificActions()
{
  //Do we have a retry set up?
  if(Phase(engine.mofs[motionID].lines[lineCounter].phase) >= Phase::Lying&& Phase(engine.mofs[motionID].lines[lineCounter].phase) <= Phase::ReduceVel&& breakCounter < engine.motionSpecificRetries)
  {
    retryTimestamp = engine.theFrameInfo.time;
    breakCounter += 1;
    state = EngineState::retryState;
    retryTime = 1000;
    if(motionID == KeyframeMotionID::back)
      lineCounter = lineCounter > 1 ? 1 : 0;
    else
      return false;
    return true;
  }
  return false;
}

bool KeyframePhase::checkRetryForFront()
{
  if(motionID == KeyframeMotionID::front && lineCounter == 2 && ratio < 0.1f && retryMotionCounter < engine.motionSpecificRetriesFront)
  {
    //Check if the ShoulderRolls could not move and are behind too much
    const bool leftArmBehind = engine.theJointAngles.angles[Joints::lShoulderPitch] > engine.shoulderPitchThreshold && engine.brokenJointData[3] <= std::abs(jointDiff1Angles.angles[3].toDegrees()) ? true : false;
    const bool rightArmBehind = engine.theJointAngles.angles[Joints::rShoulderPitch] > engine.shoulderPitchThreshold && engine.brokenJointData[9] <= std::abs(jointDiff1Angles.angles[9].toDegrees()) ? true : false;
    return leftArmBehind || rightArmBehind;
  }
  return false;
}

void KeyframePhase::isCurrentLineOver()
{
  if(!errorTriggered)
  {
    //Is the current keyframe over?
    if(ratio >= 1.f && wasRatio100Percent && (!engine.stepKeyframes || engine.theKeyStates.pressed[KeyStates::Key::chest]))
    {
      for(int i = 0; i < Joints::numOfJoints; ++i)
      {
        if(lastUnbalanced.angles[i] == JointAngles::off || lastUnbalanced.angles[i] == JointAngles::ignore || lastUnbalanced.stiffnessData.stiffnesses[i] == 0)
          lastUnbalanced.angles[i] = engine.theJointAngles.angles[i];
      }
      ratio = 0.f;
      //Do we need to do a wait time?
      if(engine.stepKeyframes || checkConditions(engine.mofs[motionID].lines[lineCounter].waitConditions, true))
        initCurrentLine();
      else
      {
        state = EngineState::waiting;
        waitTimestamp = engine.theFrameInfo.time;
      }
      //Init some values
      if(state == EngineState::working || state == EngineState::workingAction)
      {
        duration = engine.mofs[motionID].lines[lineCounter].duration;
        if(engine.stepKeyframes)
          duration = 2000;
        ratio = std::min(static_cast<float>(engine.theFrameInfo.getTimeSince(lineStartTimestamp) / duration), 1.f);
      }
    }
    //This check is done because we want to make sure to reach ratio == 1.f at least once
    wasRatio100Percent = ratio >= 1.f;
  }
  else
    OUTPUT_ERROR("Error Happend!");
}

void KeyframePhase::initGetUpMotion()
{
  ANNOTATION("KeyframeMotionEngine", motionID);
  breakUpOn = true;
  breakCounter = 0;
  isFirstLine = true;
  if(tooManyTries)
    state = EngineState::helpMeState;
  waitForFallenCheck = false;
  maxCounter = static_cast<int>(engine.mofs[motionID].lines.size());
  odometry = engine.mofs[motionID].odometryOffset;
  lineCounter = -1;
  if(motionID == KeyframeMotionID::decideAutomatic)
    OUTPUT_ERROR("decideAutomatic Motion");
  lineStartTimestamp = engine.theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
  if(!isContinueTo)
  {
    startJoints = lastJointRequest; //save the current joint angles
    lastUnbalanced.angles = lastJointRequest.angles;
    // otherwise when switching between motions, the used start angles might be 3 frames behind
    FOREACH_ENUM(Joints::Joint, joint)
    {
      if(lastJointRequest.stiffnessData.stiffnesses[joint] == 0)
      {
        startJoints.angles[joint] = engine.theJointAngles.angles[joint];
        lastUnbalanced.angles[joint] = engine.theJointAngles.angles[joint];;
      }
    }
  }
  else
    startJoints = lastUnbalanced;
  lastLineJointRequestAngles.angles = startJoints.angles;
  isInOptionalLine = false;
  if(!isContinueTo)
    lastUnbalanced.angles = engine.theJointAngles.angles;
  doBalance = engine.mofs[motionID].balanceOut;

  //decide for the first get up try if it should be mirrored or not
  if(!didFirstGetUp)
  {
    didFirstGetUp = true;
    const int frame = static_cast<int>(engine.theFrameInfo.time / static_cast<unsigned int>(1000.f * Constants::motionCycleTime));
    getUpMirror = frame % 2;
  }

  //decide if the motion front and back should be mirrored or not
  if(!isContinueTo && (motionID == KeyframeMotionID::back || motionID == KeyframeMotionID::front))
  {
    if(engine.theDamageConfigurationBody.brokenStandUp == DamageConfigurationBody::onlyMirrored
       || (engine.theDamageConfigurationBody.brokenStandUp != DamageConfigurationBody::onlyNormal && getUpMirror))
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
    initCurrentLine();
  if(isMirror && motionID != KeyframeMotionID::recoverFast)
    odometry = Pose2f(-odometry.rotation, odometry.translation.x(), -odometry.translation.y());
  //If NAO is lying on the side and will move to the right, the odometryOffset is mirrored.
  //If NAO is not lying on the side, then the odometryOffset is 0
  //TODO: recoverFast has no odometry offset. This specialCase should be for recoverFromSideBack and recoverFromSideFront
  if(motionID == KeyframeMotionID::recoverFast)
  {
    if(std::abs(engine.theInertialData.angle.x()) > 50_deg && engine.theInertialData.angle.y() > 0_deg)
      odometry = Pose2f(-odometry.rotation, odometry.translation.x(), -odometry.translation.y());
    else if(engine.theInertialData.angle.x() > -30_deg && engine.theInertialData.angle.x() < 30_deg)
      odometry = Pose2f();
  }
}

void KeyframePhase::initCurrentLine()
{
  lineStartTimestamp = engine.theFrameInfo.time - static_cast<unsigned int>(1000.f * Constants::motionCycleTime);
  startJoints = lastUnbalanced;
  lastLineJointRequestAngles.angles = startJoints.angles;
  for(unsigned int i = 0; i < numOfConditionVars; ++i)
    variableValuesCompare[i] = 0.f;
  failedWaitCounter = 0.f;
  initBalancerValues(false);
  lineCounter += 1;
  if(lineCounter == 0)
    setJointStiffnessBase();
  checkOptionalLine();
  std::fill(jointCompensationReducer.begin(), jointCompensationReducer.end(), 0.f);

  if(lineCounter < maxCounter)
  {
    //In case one joint was used for balancing on the previous keyframe but now is not anymore, the last requested angle for these joints shall be the start joints for the new keyframe.
    //Otherwise the balancing value would be missing in these joints and they would jump and damage the gears.
    std::vector<JointFactor> listY = engine.mofs[motionID].lines[lineCounter].balanceWithJoints.jointY;
    std::vector<JointFactor> listX = engine.mofs[motionID].lines[lineCounter].balanceWithJoints.jointX;

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

    lastLineJointRequestAngles.angles = startJoints.angles;
    jointsBalanceY = jointListY;
    jointsBalanceX = jointListX;

    //Is this the first keyframe of the motion? It must be a boolean and not a lineCounter == 0 check, because the first keyframe could be a conditional line.
    if(isFirstLine)
    {
      isFirstLine = false;
      if(!isContinueTo)
      {
        initBalancerValues(true);
        duration = engine.mofs[motionID].lines[lineCounter].duration;
        updateLineValues();
        for(std::size_t i = 0; i < pastJointAnglesAngles.capacity(); i++)
          pastJointAnglesAngles[i].angles.fill(convertToAngleSpecialCases(-2000.f));
        for(std::size_t i = 0; i < jointDiff1Angles.angles.size(); i++)
        {
          jointDiff1Angles.angles[i] = 0.f;
          jointDiffPredictedAngles.angles[i] = 0.f;
        }
      }
      else
      {
        duration = engine.mofs[motionID].lines[lineCounter].duration;
        updateLineValues();
        isContinueTo = false;
      }
    }

    //Override reference com of last keyframe
    if(engine.mofs[motionID].lines[lineCounter].setLastCom)
    {
      lastGoal = comDif;
      currentGoal = lastGoal + (engine.mofs[motionID].lines[lineCounter].goalCom - lastGoal) * ratio;
    }

    //Duration of the keyframe. If KeyframeMotionEngine went active while robot is still standing, go directly into stand.
    duration = engine.mofs[motionID].lines[lineCounter].duration;
    if(doFastStand && motionID == KeyframeMotionID::stand)
      duration = 300;
    if(doSlowKeyframeAfterFall)
      duration = 2000;
    doSlowKeyframeAfterFall = false;
    doFastStand = false;
    ratio = 0.f;
    balancerOn = engine.mofs[motionID].lines[lineCounter].balancerActive;

    //Get current request
    lineJointRequest.angles[Joints::headYaw] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter].head[0]);
    lineJointRequest.angles[Joints::headPitch] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter].head[1]);
    for(unsigned int i = 0; i < 6; i++)
    {
      lineJointRequest.angles[Joints::lShoulderPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter].leftArm[i]);
      lineJointRequest.angles[Joints::rShoulderPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter].rightArm[i]);
      lineJointRequest.angles[Joints::lHipYawPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter].leftLeg[i]);
      lineJointRequest.angles[Joints::rHipYawPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter].rightLeg[i]);
    }
    setJointStiffnessKeyframe();
    if(isMirror)
    {
      JointRequest mirroredJoints;
      mirroredJoints.mirror(lineJointRequest);
      lineJointRequest.angles = mirroredJoints.angles;
    }
    targetJoints = lineJointRequest;

    //This is done because if a keyframe after the first uses off or ignore angles, the angles that are calculated will be off by up to 30 degree for the first frames
    //Also the difference check of set-joints and reached-joints are to high too.
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(targetJoints.angles[i] == JointAngles::off || targetJoints.stiffnessData.stiffnesses[i] == 0)
      {
        if(!wasInWaiting)
          lineJointRequest.angles[i] = lastUnbalanced.angles[i];
        else
          lineJointRequest.angles[i] = engine.theJointAngles.angles[i];
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

void KeyframePhase::updateLineValues()
{
  //For debugging on real robot
  if(engine.stepKeyframes)
    duration = 2000;
  //Fail Save
  if(duration == 0)
    duration = 1;
  ratio = static_cast<float>(engine.theFrameInfo.getTimeSince(lineStartTimestamp) / duration);
  if(duration - engine.theFrameInfo.getTimeSince(lineStartTimestamp) < 3)  //Optimization in case we are less than 3ms away from the last frame of the current keyframe.
    ratio = 1.f;
  if(ratio > 1.f)
    ratio = 1.f;

  //Magic parameters
  goalGrowth = 0.75f * goalGrowth + 0.25f * (engine.mofs[motionID].lines[lineCounter].goalCom - lastGoal) * (static_cast<float>(1000.f * Constants::motionCycleTime) / duration);

  currentGoal = lastGoal + (engine.mofs[motionID].lines[lineCounter].goalCom - lastGoal) * ratio;
  currentBackwardCOMDiff = lastBackwardCOMDiff + (engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].backwardCOMDif - lastBackwardCOMDiff) * ratio;
  currentForwardCOMDiff = lastForwardCOMDiff + (engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].forwardCOMDif - lastForwardCOMDiff) * ratio;
  currentBackwardAngleBreakUp = lastBackwardAngleBreakUp + (engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].backwardAngleBreakUp - lastBackwardAngleBreakUp) * ratio;
  currentForwardAngleBreakUp = lastForwardAngleBreakUp + (engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].forwardAngleBreakUp - lastForwardAngleBreakUp) * ratio;

  //Angle != 0
  if(engine.mofs[motionID].clipAngle != 0)
  {
    //Is the current phase type registered to be clipped?
    if(std::find(engine.clipBreakUpAngle.begin(), engine.clipBreakUpAngle.end(), Phase(engine.mofs[motionID].lines[lineCounter].phase)) != engine.clipBreakUpAngle.end())
    {
      if(engine.mofs[motionID].clipAngle > 0)
        currentForwardAngleBreakUp.x() = lastForwardAngleBreakUp.x() + (engine.mofs[motionID].clipAngle - lastForwardAngleBreakUp.x()) * ratio;
      else
        currentBackwardAngleBreakUp.x() = lastBackwardAngleBreakUp.x() + (engine.mofs[motionID].clipAngle - lastBackwardAngleBreakUp.x()) * ratio;
    }
  }

  float mirrorFactor = 1.f;
  if(isMirror)
    mirrorFactor = -1.f;

  currentBackwardCOMDiff[1] *= mirrorFactor;
  currentForwardCOMDiff[1] *= mirrorFactor;
  currentGoal[1] *= mirrorFactor;
}

void KeyframePhase::initBalancerValues(bool calculateCurrentNew)
{
  //This is done, because if we were in a retry state, we cant take the last currentValues
  //and need to calculate them new, because our lineCounter jumped backwards.
  if(calculateCurrentNew && lineCounter >= 0 && lineCounter < maxCounter)
  {
    currentGoal = engine.mofs[motionID].lines[lineCounter].goalCom;
    currentBackwardCOMDiff = engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].backwardCOMDif;
    currentBackwardAngleBreakUp = engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].backwardAngleBreakUp;
    currentForwardCOMDiff = engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].forwardCOMDif;
    currentForwardAngleBreakUp = engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].forwardAngleBreakUp;

    //Angle != 0
    if(engine.mofs[motionID].clipAngle != 0)
    {
      //Is the current phase type registered to be clipped?
      if(std::find(engine.clipBreakUpAngle.begin(), engine.clipBreakUpAngle.end(), Phase(engine.mofs[motionID].lines[lineCounter].phase)) != engine.clipBreakUpAngle.end())
      {
        if(engine.mofs[motionID].clipAngle > 0)
          currentForwardAngleBreakUp.x() = lastForwardAngleBreakUp.x() + (engine.mofs[motionID].clipAngle - lastForwardAngleBreakUp.x()) * ratio;
        else
          currentBackwardAngleBreakUp.x() = lastBackwardAngleBreakUp.x() + (engine.mofs[motionID].clipAngle - lastBackwardAngleBreakUp.x()) * ratio;
      }
    }
  }
  lastGoal = currentGoal;
  lastBackwardCOMDiff = currentBackwardCOMDiff;
  lastBackwardAngleBreakUp = currentBackwardAngleBreakUp;
  lastForwardCOMDiff = currentForwardCOMDiff;
  lastForwardAngleBreakUp = currentForwardAngleBreakUp;
}

void KeyframePhase::waitForFallen()
{
  if(waitForFallenCheck && motionID != KeyframeMotionID::decideAutomatic)
  {
    frontHackTriggered = false;
    breakUpTimestamp = motionID != KeyframeMotionID::sitDown ? engine.theFrameInfo.time : engine.theFrameInfo.time - engine.safeFallParameters.unstiffWaitTime;
    waitForFallenCheck = false;
    //Set stiffness low and set head into a save position
    for(std::size_t i = 0; i < jointRequestOutput.angles.size(); i++)
      jointRequestOutput.stiffnessData.stiffnesses[i] = engine.safeFallParameters.bodyStiffness;
    // When falling backward
    if(engine.theInertialData.angle.y() < 0_deg)
    {
      const JointRequest copy = jointRequestOutput;
      MotionUtilities::sit(jointRequestOutput);
      MotionUtilities::safeArmsBehind(jointRequestOutput);
      if(std::abs(engine.theJointAngles.angles[Joints::lHipYawPitch]) >= engine.safeFallParameters.sitHYPThreshold) // Legs are spread out
        MotionUtilities::copy(copy, jointRequestOutput, engine.theStiffnessSettings, Joints::lHipYawPitch, Joints::numOfJoints);
      jointRequestOutput.angles[Joints::headPitch] = engine.safeFallParameters.head;
    }
    // When falling forward
    else if(engine.theInertialData.angle.y() > 0_deg)
    {
      const Angle lHYP = jointRequestOutput.angles[Joints::lHipYawPitch];
      MotionUtilities::sitFront(jointRequestOutput);
      MotionUtilities::safeArmsFront(jointRequestOutput);
      jointRequestOutput.angles[Joints::lHipYawPitch] = lHYP;
      jointRequestOutput.angles[Joints::headPitch] = -engine.safeFallParameters.head;
    }

    jointRequestOutput.angles[Joints::headYaw] = 0_deg;
    jointRequestOutput.stiffnessData.stiffnesses[Joints::headYaw] = engine.safeFallParameters.headStiffness;
    jointRequestOutput.stiffnessData.stiffnesses[Joints::headPitch] = engine.safeFallParameters.headStiffness;
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
  }
  //Go back to motion selection in case robot is (or shall start) doing get up motions
  if(motionID < KeyframeMotionID::firstNonGetUpAction || currentKeyframeMotionRequest.motion == MotionRequest::getUp)
  {
    if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > 1000)
    {
      if(tryCounter > 1)
        frontHackTriggered = true;
      state = EngineState::decideAction;
      lastNotActiveTimestamp = engine.theFrameInfo.time;
    }
  }
  //Break up happened in a non get up motion and the robot is upright again
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
  //We assume the head moved into a save position, so we can lower the stiffness
  else if(engine.theFrameInfo.getTimeSince(breakUpTimestamp) > engine.safeFallParameters.lowHeadStiffnessWaitTime)
  {
    jointRequestOutput.stiffnessData.stiffnesses[0] = engine.safeFallParameters.bodyStiffness;
    jointRequestOutput.stiffnessData.stiffnesses[1] = engine.safeFallParameters.bodyStiffness;
  }
  fastRecover = false;
  nonGetUpMotionCheck = false;
  armLeftCheck = false;
  armRightCheck = false;
  sideCheck = false;
  //todo here is a soft lock?

  //Too many tries or a malfunction in a motor was detected
  if(tryCounter >= engine.maxTryCounter || (engine.theFilteredCurrent.legMotorMalfunction && engine.motorMalfunctionBreakUp))
    state = EngineState::helpMeState;
}

void KeyframePhase::doCalibrationState()
{
  if(engine.initPhase)
  {
    switch(engine.calibrationPhase)
    {
      case Phase::HalfSplit:
        motionID = KeyframeMotionID::calibrateHalfSplit;
        break;
      case Phase::Split:
        motionID = KeyframeMotionID::calibrateSplit;
        break;
      case Phase::Sit:
        motionID = KeyframeMotionID::sit;
        break;
      case Phase::Stand:
        motionID = KeyframeMotionID::stand;
        break;
      default:
        motionID = KeyframeMotionID::decideAutomatic;
        break;
    }
    if(motionID != KeyframeMotionID::decideAutomatic)
    {
      initGetUpMotion();
      doBalance = true;
      engine.initPhase = false;
      balancerOn = false;
      fastRecover = false;
      nonGetUpMotionCheck = false;
      armLeftCheck = false;
      armRightCheck = false;
      sideCheck = false;
      duration = 2000;
      updateLineValues();
    }
  }
  if(motionID != KeyframeMotionID::decideAutomatic)
  {
    setNextJoints();
    if(ratio == 1.f && balancerOn)
    {
      pidWithCom();
      doBalanceOut();
    }
  }
}

void KeyframePhase::doManipulateOwnState()
{
  if(engine.isCalibrationState)
  {
    state = EngineState::calibrationState;
    engine.isCalibrationState = false;
  }

  if(((engine.theKeyStates.pressed[KeyStates::Key::headFront] && (engine.theKeyStates.pressed[KeyStates::Key::headMiddle] || engine.theKeyStates.pressed[KeyStates::Key::headRear]))
      || (engine.theKeyStates.pressed[KeyStates::Key::headMiddle && engine.theKeyStates.pressed[KeyStates::Key::headRear]])) && engine.theFrameInfo.getTimeSince(abortWithHeadSensorTimestamp) > 1000
     && type == MotionPhase::getUp)
  {
    abortWithHeadSensorTimestamp = engine.theFrameInfo.time;
    motionID = KeyframeMotionID::stand;
    state = EngineState::working;
    lastUnbalanced.stiffnessData.stiffnesses = lineJointRequest.stiffnessData.stiffnesses;
    lastUnbalanced = jointRequestOutput;
    initGetUpMotion();
    jointRequestOutput.stiffnessData.stiffnesses = lastUnbalanced.stiffnessData.stiffnesses;
    doBalance = false;
    balancerOn = false;
    breakUpOn = false;
  }
}

void KeyframePhase::doHelpMeStuff()
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
    motionID = KeyframeMotionID::decideAutomatic;
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

void KeyframePhase::setNextJoints()
{
  if(lineCounter < maxCounter)
  {
    float useRatio = ratio;
    calculateJointDifference();
    addJointCompensation();
    targetJoints = lineJointRequest;
    const InterpolationType interpolationType = engine.mofs[motionID].lines[lineCounter].interpolationType == InterpolationType::Default&& engine.mofs[motionID].lines[lineCounter].interpolationType != engine.mofs[motionID].interpolationType
                                                ? engine.mofs[motionID].interpolationType // use Default
                                                : (engine.mofs[motionID].lines[lineCounter].interpolationType != InterpolationType::Default
                                                   ? engine.mofs[motionID].lines[lineCounter].interpolationType // override
                                                   : InterpolationType::Linear); // fail save
    if(interpolationType == SinusMinToMax)
      useRatio = 0.5f * std::sin(ratio * Constants::pi - Constants::pi / 2.f) + 0.5f;
    else if(interpolationType == SinusZeroToMax)
      useRatio = std::sin(ratio * Constants::pi / 2);
    MotionUtilities::interpolate(startJoints, targetJoints, useRatio, jointRequestOutput, engine.theJointAngles);
    FOREACH_ENUM(Joints::Joint, joint) // otherwise ignore gets overwritten and the motionCombinator can not set the ignored joints
      jointRequestOutput.angles[joint] = targetJoints.angles[joint] == JointAngles::ignore ? JointAngles::ignore : jointRequestOutput.angles[joint];
    lastUnbalanced = jointRequestOutput;
    if(balancerOn)
      pidWithCom();
    PLOT("module:KeyframeMotionEngine:ratio", useRatio);
  }
}

void KeyframePhase::checkFinishedState()
{
  //Robot reached a default position
  if(state == EngineState::finishedRecover)
  {
    jointsBalanceY = std::vector<Joints::Joint>();
    jointsBalanceX = std::vector<Joints::Joint>();
    state = EngineState::decideAction;
    update();
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
    motionID = KeyframeMotionID::stand;
    tooManyTries = false;
    wasHelpMe = false;
    isContinueTo = false;
    waitForFallenCheck = false;
    errorTriggered = false;
    doFastStand = false;
    initGetUpMotion();
    breakUpOn = false;
    wasInWaiting = false;
    isCurrentLineOver();
    setNextJoints();
  }
  //The check for a torso angle in y-axis of less than 30deg is a fail save, in case something is wrong with the config settings
  //Motion finished and shall still balance to make sure robot will not fall when leaving the KeyframeMotionEngine
  else if(doBalance && lineCounter < maxCounter && std::abs(engine.theInertialData.angle.y()) < 30_deg)
    doBalanceOut();
  //Motion finished
  else if(engine.mofs[motionID].continueTo == KeyframeMotionID::decideAutomatic
          && ((currentKeyframeMotionRequest.motion == MotionRequest::keyframeMotion)
              || (std::abs(engine.theInertialData.angle.y()) < 30_deg && currentKeyframeMotionRequest.motion == MotionRequest::getUp)))
  {
    state = EngineState::waitForRequest;
    finishGettingUp = false;
  }
  else if(motionID != KeyframeMotionID::decideAutomatic && engine.mofs[motionID].continueTo != KeyframeMotionID::decideAutomatic)
  {
    if(engine.stepKeyframes || checkConditions(engine.mofs[motionID].lines[lineCounter].waitConditions, true))
    {
      state = lastKeyframeMotionRequest.motion == MotionRequest::getUp ? EngineState::working : EngineState::workingAction;
      doBalance = false;
      motionID = engine.mofs[motionID].continueTo;
      isContinueTo = true;
      initGetUpMotion();
      isCurrentLineOver();
      setNextJoints();
    }
    else
    {
      doBalance = false;
      state = EngineState::waiting;
      waitTimestamp = engine.theFrameInfo.time;
    }
  }
  //Seems like something went wrong
  else
  {
    doBalance = false;
    tryCounter += 1;
    state = EngineState::breakUp;
  }
}

void KeyframePhase::setJointStiffnessBase()
{
  if(lineCounter >= maxCounter || lineCounter < 0)
  {
    OUTPUT_ERROR(motionID << " " << lineCounter << "Error in setting Stiffness in KeyframeMotionEngine");
    return;
  }
  if(engine.mofs[motionID].baseLimbStiffness.size() > 0)
  {
    for(unsigned int i = 0; i < 2; i++)
      targetJoints.stiffnessData.stiffnesses[i] = engine.mofs[motionID].baseLimbStiffness[0];
    for(unsigned int i = 0; i < 6; i++)
    {
      targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::lShoulderPitch + i)) : Joints::lShoulderPitch + i] = engine.mofs[motionID].baseLimbStiffness[1];
      targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::rShoulderPitch + i)) : Joints::rShoulderPitch + i] = engine.mofs[motionID].baseLimbStiffness[2];
      targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::lHipYawPitch + i)) : Joints::lHipYawPitch + i] = engine.mofs[motionID].baseLimbStiffness[3];
      targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(Joints::Joint(Joints::rHipYawPitch + i)) : Joints::rHipYawPitch + i] = engine.mofs[motionID].baseLimbStiffness[4];
    }
    for(size_t i = 0; i < Joints::numOfJoints; i++)
      targetJoints.stiffnessData.stiffnesses[i] = !engine.stepKeyframes ? targetJoints.stiffnessData.stiffnesses[i] : std::min(engine.maxStiffnessDebugMode, targetJoints.stiffnessData.stiffnesses[i]);
    lineJointRequest.stiffnessData = targetJoints.stiffnessData;
  }
}

void KeyframePhase::setJointStiffnessKeyframe()
{
  if(lineCounter >= maxCounter || lineCounter < 0)
  {
    OUTPUT_ERROR(motionID << " " << lineCounter << "Error in setting Stiffness in KeyframeMotionEngine");
    return;
  }
  //set single motor stiffness change
  for(std::size_t i = 0; i < engine.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange.size(); i++)
    targetJoints.stiffnessData.stiffnesses[isMirror ? Joints::mirror(engine.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].joint) : engine.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].joint] =
      !engine.stepKeyframes ? engine.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].s : std::min(engine.maxStiffnessDebugMode, engine.mofs[motionID].lines[lineCounter].singleMotorStiffnessChange[i].s);
  lineJointRequest.stiffnessData = targetJoints.stiffnessData;
}

void KeyframePhase::checkOptionalLine()
{
  if(engine.stepKeyframes)
    return;
  bool doneSkipping = false;

  while(!doneSkipping && lineCounter < maxCounter)
  {
    //We are not in an optionalLine yet but found one
    if(!isInOptionalLine && engine.mofs[motionID].lines[lineCounter].conditions.size() > 0)
      doneSkipping = isInOptionalLine = checkConditions(engine.mofs[motionID].lines[lineCounter].conditions, engine.mofs[motionID].lines[lineCounter].optionalLineAnds);
    //We were in an optionalLine and found the end
    else if(isInOptionalLine && !engine.mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine)
    {
      //We found an optionalLine-ElseBlock
      if(engine.mofs[motionID].lines[lineCounter].isElseBlock)
        doneSkipping = false;
      //We found an optionalLine-IfBlock
      else if(engine.mofs[motionID].lines[lineCounter].conditions.size() > 0)
        doneSkipping = isInOptionalLine = checkConditions(engine.mofs[motionID].lines[lineCounter].conditions, engine.mofs[motionID].lines[lineCounter].optionalLineAnds);
      //We found the end of the optionalLine without a following else-Block or new optionalLine
      else
        doneSkipping = true;
    }
    //This case should never happen but is a failsafe if a case at another place in the getUpEngine is not correctly implemented
    else
      doneSkipping = isInOptionalLine || !engine.mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine;
    if(!doneSkipping)
    {
      ++lineCounter;
      while(lineCounter < maxCounter && engine.mofs[motionID].lines[lineCounter].isPartOfPreviousOptionalLine)
        ++lineCounter;
    }
  }
}

template<class C>
bool KeyframePhase::checkConditions(vector<C>& conditions, bool useAnd)
{
  static_assert(std::is_assignable<Condition, C>::value, "Incompatible types!");

  std::size_t conditionSize = conditions.size();
  if(conditionSize == 0)
    return true;
  //Write the values for the conditions
  float variableValues[numOfConditionVars];
  variableValues[ConditionVar(InertialDataAngleY)] = engine.theInertialData.angle.y().toDegrees();
  variableValues[ConditionVar(FluctuationY)] = fluctuation.y().toDegrees() * 1.f / Constants::motionCycleTime;
  const float leftArmValue = engine.brokenJointData[2] <= std::abs(jointDiff1Angles.angles[2].toDegrees()) || engine.brokenJointData[3] <= std::abs(jointDiff1Angles.angles[3].toDegrees()) || engine.brokenJointData[4] <= std::abs(jointDiff1Angles.angles[4].toDegrees()) || engine.brokenJointData[5] <= std::abs(jointDiff1Angles.angles[5].toDegrees()) ? 0.f : 1.f;
  const float rightArmValue = engine.brokenJointData[8] <= std::abs(jointDiff1Angles.angles[8].toDegrees()) || engine.brokenJointData[9] <= std::abs(jointDiff1Angles.angles[9].toDegrees()) || engine.brokenJointData[10] <= std::abs(jointDiff1Angles.angles[10].toDegrees()) || engine.brokenJointData[11] <= std::abs(jointDiff1Angles.angles[11].toDegrees()) ? 0.f : 1.f;
  variableValues[ConditionVar(BrokenLeftArm)] = isMirror ? rightArmValue : leftArmValue;
  variableValues[ConditionVar(BrokenRightArm)] = isMirror ? leftArmValue : rightArmValue;
  variableValues[ConditionVar(WaitTime)] = static_cast<float>(engine.theFrameInfo.getTimeSince(waitTimestamp));
  variableValues[ConditionVar(FrontHack)] = frontHackTriggered;
  variableValues[ConditionVar(IsSitting)] = std::min(engine.theJointAngles.angles[Joints::lKneePitch], engine.theJointAngles.angles[Joints::rKneePitch]).toDegrees();
  variableValues[ConditionVar(HYPDifference)] = std::abs(engine.theJointAngles.angles[Joints::lHipYawPitch].toDegrees() - jointRequestOutput.angles[Joints::lHipYawPitch].toDegrees());
  variableValues[ConditionVar(FootSupportVal)] = (!isMirror ? 1.f : -1.f) * engine.theFootSupport.support;

  //If a waitCondition is checked, this will help to abort the wait time IF it is impossible to fulfill the condition.
  //If a normal condition is checked, this code just wastes processor time and does stuff that has no consequences.
  if(!engine.mofs[motionID].lines[lineCounter].forbidWaitBreak)
  {
    for(std::size_t i = 0; i < conditionSize; i++)
    {
      if(variableValues[conditions[i].variable] <= conditions[i].lowerFloat)
      {
        float diff = variableValues[conditions[i].variable] - conditions[i].lowerFloat;
        if(diff < variableValuesCompare[conditions[i].variable])
          failedWaitCounter += 100.f * Constants::motionCycleTime;
        variableValuesCompare[conditions[i].variable] = diff;
      }
      else if(variableValues[conditions[i].variable] > conditions[i].higherFloat)
      {
        float diff = variableValues[conditions[i].variable] - conditions[i].higherFloat;
        if(diff > variableValuesCompare[conditions[i].variable])
          failedWaitCounter += 100.f * Constants::motionCycleTime;
        variableValuesCompare[conditions[i].variable] = diff;
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

template bool KeyframePhase::checkConditions(std::vector<Condition>& conditions, bool useAnd);
template bool KeyframePhase::checkConditions(std::vector<WaitCondition>& conditions, bool useAnd);

void KeyframePhase::pidWithCom()
{
  if(lineCounter > maxCounter || KeyframeMotionID(motionID) == KeyframeMotionID::decideAutomatic)
  {
    OUTPUT_ERROR("LineCounter or motionID is wrong in pidWithCom!");
    return;
  }
  if(engine.theFrameInfo.getTimeSince(lastPIDCom) > 20)
    valPID_I = valPID_D = valPID_P = balanceValue = Vector2f::Zero();
  //Calc base PID values
  lastPIDCom = engine.theFrameInfo.time;
  KeyframeMotionParameters::BalanceFactors factors = engine.theKeyframeMotionParameters.balanceList[Phase(engine.mofs[motionID].lines[lineCounter].phase)].balanceFactor;
  constexpr float cycleTime = Constants::motionCycleTime;
  valPID_D = (comDif - oldCom) - goalGrowth;
  valPID_P = (comDif - currentGoal) * 1.f;
  valPID_I += comDif - currentGoal;

  //calc values for forward/backward movement
  const float nQuadMinY = currentBackwardCOMDiff.x() == 0 ? 1.f : 1 / std::pow(currentBackwardCOMDiff.x(), factors.powDegreeBack.x());
  const float nQuadMaxY = currentForwardCOMDiff.x() == 0 ? 1.f : 1 / std::pow(currentForwardCOMDiff.x(), factors.powDegreeFront.x());

  const float factorY = comDif.x() - currentGoal.x() > 0
                        ? std::min(nQuadMaxY * std::pow(comDif.x() - currentGoal.x(), factors.powDegreeFront.x()), factors.increaseFactorCapFront.x())
                        : std::min(nQuadMinY * std::pow(comDif.x() - currentGoal.x(), factors.powDegreeBack.x()), factors.increaseFactorCapBack.x());

  const float pidPY = valPID_P.x() * (factors.borderPIDPX[0] + std::min(factorY, factors.borderPIDPX[1]));
  const float pidIY = valPID_I[0] * 0.02f;
  const float pidDY = (valPID_D[0] / cycleTime) * (factors.borderPIDDX[0] + std::min(factorY, factors.borderPIDDX[1]));

  const float balanceValueY = (pidPY + pidIY + pidDY) * cycleTime;
  const float diffYdiff = balanceValueY - balanceValue[1];
  balanceValue[1] += diffYdiff < 0 ? std::max(0.4f * diffYdiff, -1.f) : std::min(0.4f * diffYdiff, 1.f); //clip balance value to max 1 degree change per frame

  //calc values for sideways movement
  //TODO side balancing is not really needed and could be removed
  const float nQuadMinX = currentBackwardCOMDiff.y() == 0 ? 1.f : 1 / std::pow(currentBackwardCOMDiff.y(), factors.powDegreeBack.y());
  const float nQuadMaxX = currentForwardCOMDiff.y() == 0 ? 1.f : 1 / std::pow(currentForwardCOMDiff.y(), factors.powDegreeFront.y());

  const float factorX = comDif.y() - currentGoal.y() > 0
                        ? std::min(nQuadMaxX * std::pow(comDif.y() - currentGoal.y(), factors.powDegreeFront.y()), factors.increaseFactorCapFront.y())
                        : std::min(nQuadMinX * std::pow(comDif.y() - currentGoal.y(), factors.powDegreeBack.y()), factors.increaseFactorCapBack.y());

  const float pidPX = valPID_P.y() * (factors.borderPIDPY[0] + std::min(factorX, factors.borderPIDPY[1]));
  const float pidIX = valPID_I[1] * 0.01f;
  const float pidDX = (valPID_D[1] / cycleTime) * (factors.borderPIDDY[0] + std::min(factorX, factors.borderPIDDY[1]));

  const float balanceValueX = (pidPX + pidIX + pidDX) * cycleTime;
  const float diffXdiff = balanceValueX - balanceValue[0];
  balanceValue[0] += diffXdiff < 0 ? std::max(0.2f * diffXdiff, -2.f) : std::min(0.2f * diffXdiff, 2.f); //clip balance value to max 2 degree change per frame

  //save D-part value for balanceOut function
  pidDForward = pidDY;

  //apply balance values
  if(!engine.stepKeyframes && state != EngineState::calibrationState && balancerOn)
    addBalanceFactor(balanceValue[1], balanceValue[0]);

  PLOT("module:KeyframeMotionEngine:pid:p", pidPY);
  PLOT("module:KeyframeMotionEngine:pid:i", pidIY);
  PLOT("module:KeyframeMotionEngine:pid:d", pidDY);
  PLOT("module:KeyframeMotionEngine:pid:borderF", currentGoal.x() + currentBackwardCOMDiff.x());
  PLOT("module:KeyframeMotionEngine:pid:borderB", currentGoal.x() + currentForwardCOMDiff.x());
  PLOT("module:KeyframeMotionEngine:pid:com", comDif.x());
  PLOT("module:KeyframeMotionEngine:pid:balance", balanceValue[1]);
  PLOT("module:KeyframeMotionEngine:pid:angleY", engine.theInertialData.angle.y().toDegrees());
  PLOT("module:KeyframeMotionEngine:pid:angleX", engine.theInertialData.angle.x().toDegrees());
}

void KeyframePhase::addBalanceFactor(float factorY, float factorX)
{
  for(JointFactor jointList : engine.mofs[motionID].lines[lineCounter].balanceWithJoints.jointY)
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
    if(targetJoints.angles[joint] == JointAngles::off || targetJoints.angles[joint] == JointAngles::ignore || targetJoints.stiffnessData.stiffnesses[joint] == 0)
      lineJointRequest.angles[joint] = lastUnbalanced.angles[joint];
    const Rangef qRange(engine.theJointLimits.limits[joint].min, engine.theJointLimits.limits[joint].max);
    jointRequestOutput.angles[joint] += convertToAngleSpecialCases(factorY * factor);
    jointRequestOutput.angles[joint] = qRange.limit(jointRequestOutput.angles[joint]);
  }
  //////////////////
  for(JointFactor jointList : engine.mofs[motionID].lines[lineCounter].balanceWithJoints.jointX)
  {
    Joints::Joint joint = jointList.joint;
    float factor = jointList.factor;
    if(isMirror)
      joint = Joints::mirror(jointList.joint);
    if((joint == Joints::lAnklePitch
        || joint == Joints::rAnklePitch) && isMirror)
      factor *= -1.f;
    if(targetJoints.angles[joint] == JointAngles::off || targetJoints.angles[joint] == JointAngles::ignore || targetJoints.stiffnessData.stiffnesses[joint] == 0)
      lineJointRequest.angles[joint] = lastUnbalanced.angles[joint];
    const Rangef qRange(engine.theJointLimits.limits[joint].min, engine.theJointLimits.limits[joint].max);

    jointRequestOutput.angles[joint] += convertToAngleSpecialCases(factorX * factor);
    jointRequestOutput.angles[joint] = qRange.limit(jointRequestOutput.angles[joint]);
  }
}

void KeyframePhase::addJointCompensation()
{
  //Reduce jointCompensation of last Keyframe
  //This reduces the joint compensation of the keyframe before, to prevent an overcompensation.
  //Otherwise the compensation of the keyframe before would be reduce not fast enough.
  //Compensation reduction does not work if used with negative values, because every use-case is with positive values.
  const JointRequest lastTargetConfig = calculatedCurrentJointRequestWithoutBalance(); //Joint targets of last Keyframe without balancing and compensation
  JointRequest startDif;
  for(unsigned int i = 0; i < Joints::numOfJoints; i++)
    startDif.angles[i] = lastTargetConfig.angles[i] - lastLineJointRequestAngles.angles[i]; //Dif of joint targets of last Keyframe with and without balancing and compensation
  if(lineCounter > 0)
  {
    //save last compensation reduce factors
    const auto jointCompensationReducerCopy = jointCompensationReducer;
    if(!engine.mofs[motionID].lines[lineCounter - 1].jointCompensation.empty())
      //jointCompensation index must be 0, because the frameWork assumes, that only(!) the first entry is used.
      for(JointCompensationParams list : engine.mofs[motionID].lines[lineCounter - 1].jointCompensation[0].jointCompensationParams)
      {
        //we compensate the asymmetry in the current motion. no need to handle overcompensation
        if(list.hipPitchDifferenceCompensation)
          continue;

        //If joint started to move again then increase compensationReducer
        if(jointDiffPredictedAngles.angles[list.jointDelta] < jointDiff1Angles.angles[list.jointDelta]
           && jointDiff1Angles.angles[list.jointDelta] - jointDiffPredictedAngles.angles[list.jointDelta] > engine.minJointCompensationReduceAngleDiff)
          jointCompensationReducer[list.jointDelta] += engine.mofs[motionID].lines[lineCounter].jointCompensation[0].reduceFactorJointCompensation;

        //Update the compensation and remove them from the startJoints.
        for(JointPair jointPair : list.jointPairs)
        {
          const Joints::Joint joint = jointPair.joint;
          //We do not need to check the predictJointDif flag, because jointDifPredicted values already do this job
          if(jointDiffPredictedAngles.angles[list.jointDelta] < jointDiff1Angles.angles[list.jointDelta] //stuck joint moves fast
             && jointDiff1Angles.angles[list.jointDelta] - jointDiffPredictedAngles.angles[list.jointDelta] > engine.minJointCompensationReduceAngleDiff //stuck joint moves fast enough (diff > threshold)
             && lastTargetConfig.angles[joint] != JointAngles::off && lastTargetConfig.angles[joint] != JointAngles::ignore && targetJoints.stiffnessData.stiffnesses[joint] != 0//special case
             && lastLineJointRequestAngles.angles[joint] != JointAngles::off && lastLineJointRequestAngles.angles[joint] != JointAngles::ignore //special case
             && jointCompensationReducer[list.jointDelta] <= 1.f && jointCompensationReducerCopy[list.jointDelta] < jointCompensationReducer[list.jointDelta])// make sure we don't erase more than 100% of the last jointCompensation and only reduce if the joint keeps moving.
            startJoints.angles[joint] += engine.mofs[motionID].lines[lineCounter].jointCompensation[0].reduceFactorJointCompensation * startDif.angles[joint];
        }
      }
  }
  //Compensation reduction end

  //Add jointCompensation for the current keyframe
  lineJointRequest.angles = lineJointRequest2Angles.angles;
  if(!engine.mofs[motionID].lines[lineCounter].jointCompensation.empty())
    for(JointCompensationParams list : engine.mofs[motionID].lines[lineCounter].jointCompensation[0].jointCompensationParams)
    {
      Joints::Joint jointDelta = list.jointDelta;
      if(isMirror)
        jointDelta = Joints::mirror(jointDelta);
      for(JointPair jointPair : list.jointPairs)
      {
        float percent = 0;
        //if minVal is below 0, maxVal is not allowed to be above 0
        //if maxVal is above 0, minVal is not allowed to be below 0
        const bool aboveCase = list.minVal >= 0 && list.maxVal >= 0;
        const bool belowCase = list.maxVal <= 0 && list.minVal < 0;
        //when hipPitch difference compensation is active, then we can not use the predicted joint angles
        float refAngle = !list.hipPitchDifferenceCompensation ? jointDiffPredictedAngles.angles[jointDelta].toDegrees() : Angle(engine.theJointAngles.angles[Joints::lHipPitch] - engine.theJointAngles.angles[Joints::rHipPitch]).toDegrees();
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

void KeyframePhase::calculateCOMInSupportPolygon()
{
  //Offset for the foot area
  std::vector<Pose3f> footOffsets;
  footOffsets.emplace_back(engine.supportPolygonOffsets.x(), 0.f, 0.f);
  footOffsets.emplace_back(engine.supportPolygonOffsets.y(), 0.f, 0.f);
  footOffsets.emplace_back(0.f, -engine.supportPolygonOffsets.z(), 0.f);
  footOffsets.emplace_back(0.f, engine.supportPolygonOffsets.z(), 0.f);

  //rotation matrix
  const Pose3f gyroInTorso(RotationMatrix(Rotation::AngleAxis::unpack(Vector3f(-engine.theInertialData.angle.x(), -engine.theInertialData.angle.y(), 0.f))));

  //get all foot points based on the offsets
  std::vector<Pose3f> limbs;
  for(unsigned i = 0; i < 4; i++)
    limbs.emplace_back(gyroInTorso.inverse() * engine.theRobotModel.soleLeft * footOffsets[i]);
  for(unsigned i = 0; i < 4; i++)
    limbs.emplace_back(gyroInTorso.inverse() * engine.theRobotModel.soleRight * footOffsets[i]);

  //sort them by z coordinate
  std::sort(limbs.begin(), limbs.end(),
            [](const auto & l1, const auto & l2)
  {
    return l1.translation.z() < l2.translation.z();
  });
  const float groundHeight = limbs[0].translation.z();
  std::vector<Pose3f> supportPolygon;

  //sort all foot point out, that are too far above the ground. these points have no ground contact.
  for(Pose3f& p : limbs)
    if(p.translation.z() < groundHeight + 100.f)
      supportPolygon.emplace_back(p);
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

  const float minY = supportPolygon[0].translation.y();
  const float maxY = supportPolygon[supportPolygon.size() - 1].translation.y();

  //sort by x
  std::sort(supportPolygon.begin(), supportPolygon.end(),
            [](const Pose3f& p1, const Pose3f& p2) -> bool
  {
    return p1.translation.x() < p2.translation.x();
  });

  const float minX = supportPolygon[0].translation.x();
  const float maxX = supportPolygon[supportPolygon.size() - 1].translation.x();

  //sort by z
  std::sort(supportPolygon.begin(), supportPolygon.end(),
            [](const Pose3f& p1, const Pose3f& p2) -> bool
  {
    return p1.translation.z() < p2.translation.z();
  });

  float z = supportPolygon[0].translation.z();

  //save all 4 point of the foot area
  std::vector<Pose3f> supportPolygonCopy;
  const Pose3f a = Pose3f(minX, minY, z);
  const Pose3f b = Pose3f(maxX, minY, z);
  const Pose3f c = Pose3f(minX, maxY, z);
  const Pose3f d = Pose3f(maxX, maxY, z);
  supportPolygonCopy.emplace_back(a);
  supportPolygonCopy.emplace_back(b);
  supportPolygonCopy.emplace_back(d);
  supportPolygonCopy.emplace_back(c);

  //calculate middle point of foot area
  supportCenter += a.translation.array();
  supportCenter += b.translation.array();
  supportCenter += c.translation.array();
  supportCenter += d.translation.array();
  supportCenter.translation.array() /= 4;

  const Pose3f comInGyro = gyroInTorso.inverse() * engine.theRobotModel.centerOfMass; //com in gyro
  const Vector3f balanceDif = comInGyro.translation - supportCenter.translation;//difference of com in torso and middle point in foot area. z-axis is ignored

  oldCom2 = oldCom;
  oldCom = comDif;
  comDif = { balanceDif.x(), balanceDif.y() };

  if(engine.calculateDrawing)
  {
    Pose3f supportCenterInTorso = gyroInTorso * supportCenter; //middle point in torso
    Pose3f comInSupport = gyroInTorso.inverse() * engine.theRobotModel.centerOfMass;
    comInSupport.translation.z() = supportCenterInTorso.translation.z();
    comInSupport = gyroInTorso * comInSupport;
    LINE3D("module:KeyframeMotionEngineOutput:coordinates", comInSupport.translation.x(), comInSupport.translation.y(), comInSupport.translation.z(), engine.theRobotModel.centerOfMass.x(), engine.theRobotModel.centerOfMass.y(), engine.theRobotModel.centerOfMass.z(), 4, ColorRGBA::red);
    std::vector<Pose3f> limbsInTorso;
    for(const Pose3f& l : supportPolygon)
      limbsInTorso.emplace_back(gyroInTorso * l);
    for(unsigned i = 0; i < limbsInTorso.size(); i++)
      LINE3D("module:KeyframeMotionEngineOutput:coordinates", limbsInTorso[i].translation.x(), limbsInTorso[i].translation.y(), limbsInTorso[i].translation.z(), limbsInTorso[(i + 1) % limbsInTorso.size()].translation.x(), limbsInTorso[(i + 1) % limbsInTorso.size()].translation.y(), limbsInTorso[(i + 1) % limbsInTorso.size()].translation.z(), 4, ColorRGBA::blue);
    CROSS3D("module:KeyframeMotionEngineOutput:coordinates", supportCenterInTorso.translation.x(), supportCenterInTorso.translation.y(), supportCenterInTorso.translation.z(), 3, 5, ColorRGBA::orange);
  }
}

JointRequest KeyframePhase::calculatedCurrentJointRequestWithoutBalance()
{
  if(lineCounter > 0)
  {
    JointRequest lastTarget;
    lastTarget.angles[Joints::headYaw] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter - 1].head[0]);
    lastTarget.angles[Joints::headPitch] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter - 1].head[1]);
    for(unsigned int i = 0; i < 6; i++)
    {
      lastTarget.angles[Joints::lShoulderPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter - 1].leftArm[i]);
      lastTarget.angles[Joints::rShoulderPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter - 1].rightArm[i]);
      lastTarget.angles[Joints::lHipYawPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter - 1].leftLeg[i]);
      lastTarget.angles[Joints::rHipYawPitch + i] = convertToAngleSpecialCases(engine.mofs[motionID].lines[lineCounter - 1].rightLeg[i]);
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

void KeyframePhase::updateSensorArray()
{
  lastTorsoAngle.push_front(engine.theInertialData.angle.head<2>());

  const Vector2a vel = lastTorsoAngle[0] - lastTorsoAngle[1];
  const Vector2a acc = vel - (lastTorsoAngle[1] - lastTorsoAngle[2]);

  fluctuation = Vector2a(std::abs(acc.x()), std::abs(acc.y())) + Vector2a(std::abs(vel.x()), std::abs(vel.y()));

  if(lineCounter < maxCounter && state > EngineState::decideAction && maxCounter > 0)
    updateLineValues();
  calculateCOMInSupportPolygon();
}

void KeyframePhase::doBalanceOut()
{
  if(doBalance)
  {
    if(state != EngineState::calibrationState)
      state = EngineState::balanceOut;
    doBalance = false;
    balancerOn = true;
    goalGrowth = Vector2f::Zero();
    doBalanceOutTimestamp = engine.theFrameInfo.time;
  }
  ratio = 1.f;
  if(state != EngineState::calibrationState)
  {
    if(!isInBreakUpRange())
    {
      if(engine.theFrameInfo.getTimeSince(doBalanceOutTimestamp) > engine.balanceOutParams.maxTime
         || (std::abs(fluctuation.y()) < engine.balanceOutParams.minFluctuation
             && engine.theInertialData.angle.y() < engine.balanceOutParams.minForwardAngle
             && engine.theInertialData.angle.y() > engine.balanceOutParams.minBackwardAngle
             && std::abs(pidDForward) < engine.balanceOutParams.minPIDDValue))
      {
        state = EngineState::finished;
        finishGettingUp = false;
        // TODO until the full motion refactoring finished, the chance of falling robots after finishing get ups is like 1% higher....
      }
      if(state != EngineState::finished && state != EngineState::breakUp)
        setNextJoints();
      else if(state == EngineState::finished)
        checkFinishedState();
    }
  }
}

Angle KeyframePhase::convertToAngleSpecialCases(float angle)
{
  return angle == JointAngles::off || angle == JointAngles::ignore ? static_cast<Angle>(angle) : Angle::fromDegrees(angle);
}

/*
 * Example to use this feature. This example should still result in a successful get up try (most of the time)
set module:KeyframeMotionEngine:setLineCounterFailureStart 1
set module:KeyframeMotionEngine:setLineCounterFailureEnd 4
set module:KeyframeMotionEngine:setMotionIDFailure fromSplit
set module:KeyframeMotionEngine:setRatioFailureStart 0.6
set module:KeyframeMotionEngine:setRatioFailureEnd 0
set module:KeyframeMotionEngine:setJointFailure lHipYawPitch
dr module:KeyframeMotionEngine:addMotionFailure
 */
void KeyframeMotionEngine::addJointFailureBehavior(KeyframeMotionID motionIDFailure, int lineCounterFailureStart,
                                                   float ratioFailureStart, int lineCounterFailureEnd, float ratioFailureEnd,
                                                   Joints::Joint joint, float reduceRatio)
{
  JointFailureParams params;
  params.motionIDFailure = motionIDFailure;
  params.lineCounterFailureStart = lineCounterFailureStart;
  params.lineCounterFailureEnd = lineCounterFailureEnd;
  params.ratioStart = ratioFailureStart;
  params.ratioEnd = ratioFailureEnd;
  params.joint = joint;
  params.reduceRatio = reduceRatio;
  jointFailureList.emplace_back(params);
}

void KeyframeMotionEngine::clearJointFailureBehavior()
{
  jointFailureList.resize(0);
}

void KeyframePhase::checkJointFailureBehavior()
{
  jointRequestWithoutFailureBehaviorAngles.angles = jointRequestOutput.angles;
  //Do we have a request to simulate a joint failure?
  if(engine.jointFailureList.size() > 0)
  {
    for(JointFailureParams& params : engine.jointFailureList)
    {
      //Is the joint failure active?
      if(motionID == params.motionIDFailure
         && (lineCounter > params.lineCounterFailureStart
             || (lineCounter == params.lineCounterFailureStart && ratio >= params.ratioStart)))
      {
        Joints::Joint joint = params.joint == Joints::lHipYawPitch || !isMirror ? params.joint : Joints::mirror(params.joint);
        if(lineCounter < params.lineCounterFailureEnd || (lineCounter == params.lineCounterFailureEnd && ratio <= params.ratioEnd))
        {
          jointFailureRequestCurrentBordersAngles = jointFailureRequestAngles;
          jointRequestOutput.angles[joint] = jointFailureRequestAngles.angles[joint];
          params.reduceRatio = 0.f;
        }
        else
        {
          params.reduceRatio += engine.jointCompensationSimulationFactor;
          params.reduceRatio = std::min(params.reduceRatio, 1.f);
          jointRequestOutput.angles[joint] = jointRequestOutput.angles[joint] * params.reduceRatio + (1 - params.reduceRatio) * jointFailureRequestCurrentBordersAngles.angles[joint];
        }
      }
    }
  }
}

void KeyframeMotionEngine::editMode(KeyframeMotionGenerator& output)
{
#ifndef NDEBUG
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:addMotionFailure")
    addJointFailureBehavior(motionIDFailure, lineCounterFailureStart, ratioFailureStart, lineCounterFailureEnd, ratioFailureEnd, joint, 0.f);
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:clearJointFailure")
    clearJointFailureBehavior();
#endif

  //TODO at some point in the future: do a view with this code
  MODIFY_ONCE("module:KeyframeMotionEngine:Edit:setLine", output.editKeyframe.line);
  MODIFY_ONCE("module:KeyframeMotionEngine:Edit:setMotion", output.editKeyframe.motion);
  int line = output.editKeyframe.line;
  KeyframeMotionID motion = output.editKeyframe.motion;

  //Adds a keyframe to the motion at the end
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:addLine")
  {
    if(motion == KeyframeMotionID::decideAutomatic)
    {
      OUTPUT_TEXT("You can not edit motion decideAutomatic!");
      return;
    }
    mofs[motion].lines.emplace_back();
    output.editKeyframe.line = static_cast<int>(mofs[motion].lines.size()) - 1;
    if(mofs[motion].lines.size() >= 2)
    {
      mofs[motion].lines[output.editKeyframe.line].head = mofs[motion].lines[output.editKeyframe.line - 1].head;
      mofs[motion].lines[output.editKeyframe.line].leftArm = mofs[motion].lines[output.editKeyframe.line - 1].leftArm;
      mofs[motion].lines[output.editKeyframe.line].rightArm = mofs[motion].lines[output.editKeyframe.line - 1].rightArm;
      mofs[motion].lines[output.editKeyframe.line].leftLeg = mofs[motion].lines[output.editKeyframe.line - 1].leftLeg;
      mofs[motion].lines[output.editKeyframe.line].rightLeg = mofs[motion].lines[output.editKeyframe.line - 1].rightLeg;
    }
  }

  //Adds a keyframe to the motion after "line"
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:addLineAfter")
  {
    if(!checkIfEditIsPossible(line, motion))
    {
      OUTPUT_TEXT("addLineAfter Failed!");
      return;
    }
    std::vector<MofLine> copyLines;
    for(int i = 0; i < static_cast<int>(mofs[motion].lines.size()); i++)
    {
      copyLines.emplace_back(mofs[motion].lines[i]);
      if(i == line)
        copyLines.emplace_back();
    }
    mofs[motion].lines = copyLines;
    output.editKeyframe.line += 1;
    if(mofs[motion].lines.size() >= 2)
    {
      mofs[motion].lines[output.editKeyframe.line].head = mofs[motion].lines[output.editKeyframe.line - 1].head;
      mofs[motion].lines[output.editKeyframe.line].leftArm = mofs[motion].lines[output.editKeyframe.line - 1].leftArm;
      mofs[motion].lines[output.editKeyframe.line].rightArm = mofs[motion].lines[output.editKeyframe.line - 1].rightArm;
      mofs[motion].lines[output.editKeyframe.line].leftLeg = mofs[motion].lines[output.editKeyframe.line - 1].leftLeg;
      mofs[motion].lines[output.editKeyframe.line].rightLeg = mofs[motion].lines[output.editKeyframe.line - 1].rightLeg;
    }
  }

  //Adds a keyframe to the motion before "line"
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:addLineBefore")
  {
    if(!checkIfEditIsPossible(line, motion))
    {
      OUTPUT_TEXT("addLineBefore Failed!");
      return;
    }
    std::vector<MofLine> copyLines;
    for(int i = 0; i < static_cast<int>(mofs[motion].lines.size()); i++)
    {
      if(i == line)
        copyLines.emplace_back();
      copyLines.emplace_back(mofs[motion].lines[i]);
    }
    mofs[motion].lines = copyLines;
    if(mofs[motion].lines.size() >= 2)
    {
      mofs[motion].lines[output.editKeyframe.line].head = mofs[motion].lines[output.editKeyframe.line + 1].head;
      mofs[motion].lines[output.editKeyframe.line].leftArm = mofs[motion].lines[output.editKeyframe.line + 1].leftArm;
      mofs[motion].lines[output.editKeyframe.line].rightArm = mofs[motion].lines[output.editKeyframe.line + 1].rightArm;
      mofs[motion].lines[output.editKeyframe.line].leftLeg = mofs[motion].lines[output.editKeyframe.line + 1].leftLeg;
      mofs[motion].lines[output.editKeyframe.line].rightLeg = mofs[motion].lines[output.editKeyframe.line + 1].rightLeg;
    }
  }

  //Deletes the keyframe at "line" of the motion
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:deleteLine")
  {
    if(!checkIfEditIsPossible(line, motion))
    {
      OUTPUT_TEXT("deleteLine Failed!");
      return;
    }
    std::vector<MofLine> copyLines;
    for(int i = 0; i < static_cast<int>(mofs[motion].lines.size()); i++)
    {
      if(i != line)
        copyLines.emplace_back(mofs[motion].lines[i]);
    }
    mofs[motion].lines = copyLines;
  }

  //Loads the current measured joint values
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:getCurrentAngles")
    output.editKeyframe.angles = theJointAngles;

  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:getCurrentAnglesLeftArm")
    for(size_t i = Joints::firstArmJoint; i < Joints::firstRightArmJoint; i++)
      output.editKeyframe.angles.angles[i] = theJointAngles.angles[i];

  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:getCurrentAnglesRightArm")
    for(size_t i = Joints::firstRightArmJoint; i < Joints::firstLegJoint; i++)
      output.editKeyframe.angles.angles[i] = theJointAngles.angles[i];

  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:getCurrentAnglesLeftLeg")
    for(size_t i = Joints::firstLegJoint; i < Joints::firstRightLegJoint; i++)
      output.editKeyframe.angles.angles[i] = theJointAngles.angles[i];

  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:getCurrentAnglesRightLeg")
    for(size_t i = Joints::firstRightLegJoint; i < Joints::numOfJoints; i++)
      output.editKeyframe.angles.angles[i] = theJointAngles.angles[i];

  //Overrides the joint values in the config with the one in the representation
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:Edit:setCurrentAngles")
  {
    if(!checkIfEditIsPossible(line, motion))
    {
      OUTPUT_TEXT("setCurrentAngles Failed!");
      return;
    }
    for(size_t i = 0; i < Joints::numOfJoints; i++)
    {
      mofs[motion].lines[line].head[0] = output.editKeyframe.angles.angles[Joints::headYaw].toDegrees();
      mofs[motion].lines[line].head[1] = output.editKeyframe.angles.angles[Joints::headPitch].toDegrees();
      for(unsigned int i = 0; i < 6; i++)
      {
        mofs[motion].lines[line].leftArm[i] = output.editKeyframe.angles.angles[Joints::lShoulderPitch + i].toDegrees();
        mofs[motion].lines[line].rightArm[i] = output.editKeyframe.angles.angles[Joints::rShoulderPitch + i].toDegrees();
        mofs[motion].lines[line].leftLeg[i] = output.editKeyframe.angles.angles[Joints::lHipYawPitch + i].toDegrees();
        mofs[motion].lines[line].rightLeg[i] = output.editKeyframe.angles.angles[Joints::rHipYawPitch + i].toDegrees();
      }
    }
  }
}

bool KeyframeMotionEngine::checkIfEditIsPossible(int line, KeyframeMotionID motion)
{
  if(line < 0)
    return false;
  if(motion == KeyframeMotionID::decideAutomatic)
    return false;
  if(line >= static_cast<int>(mofs[motion].lines.size()))
    return false;
  return true;
}

MAKE_MODULE(KeyframeMotionEngine, motionControl);
