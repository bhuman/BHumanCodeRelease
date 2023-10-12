/*
 * @file KeyframeMotionEngine.cpp
 * @author Philip Reichenberg
 */

#include "KeyframeMotionEngine.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"

KeyframeMotionEngine::KeyframeMotionEngine()
{
  // Check if both enums have the same number
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::sitDown) > static_cast<std::size_t>(KeyframeMotionID::sitDown));
  ASSERT((KeyframeMotionListID::numOfKeyframeMotionListIDs - KeyframeMotionListID::sitDown) == (KeyframeMotionID::numOfKeyframeMotionIDs - KeyframeMotionID::firstNonGetUpAction));
  ASSERT((KeyframeMotionListID::numOfKeyframeMotionListIDs - KeyframeMotionListID::sitDown) == 7);
  [[maybe_unused]] const std::size_t offset = static_cast<std::size_t>(KeyframeMotionListID::sitDown) - static_cast<std::size_t>(KeyframeMotionID::firstNonGetUpAction);
  ASSERT(static_cast<std::size_t>(KeyframeMotionID::firstNonGetUpAction) == static_cast<std::size_t>(KeyframeMotionID::sitDown));
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::sitDown) == static_cast<std::size_t>(KeyframeMotionID::sitDown) + offset);
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::sitDownKeeper) == static_cast<std::size_t>(KeyframeMotionID::sitDown + 1) + offset);
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::keeperJumpLeft) == static_cast<std::size_t>(KeyframeMotionID::sitDown + 2) + offset);
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::genuflectStand) == static_cast<std::size_t>(KeyframeMotionID::sitDown + 3) + offset);
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::genuflectStandDefender) == static_cast<std::size_t>(KeyframeMotionID::sitDown + 4) + offset);
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::demoBannerWave) == static_cast<std::size_t>(KeyframeMotionID::sitDown + 5) + offset);
  ASSERT(static_cast<std::size_t>(KeyframeMotionListID::demoBannerWaveInitial) == static_cast<std::size_t>(KeyframeMotionID::sitDown + 6) + offset);
}

KeyframePhase::KeyframePhase(KeyframeMotionEngine& engine, const KeyframeMotionRequest& keyframeMotionRequest, const MotionPhase& lastPhase) :
  KeyframePhaseBase(engine, keyframeMotionRequest)
{
  bool initStartJoints = true;
  if(lastPhase.type == MotionPhase::getUp || lastPhase.type == MotionPhase::keyframeMotion)
  {
    const auto& lastKeyframePhase = static_cast<const KeyframePhase&>(lastPhase);
    lastKeyframeMotionRequest = lastKeyframePhase.currentKeyframeMotionRequest;
    lastJointRequest = lastKeyframePhase.jointRequestOutput;
    lastUnbalanced = lastKeyframePhase.lastUnbalanced;
    jointRequestOutput = lastKeyframePhase.jointRequestOutput;
    currentKeyframe = lastKeyframePhase.currentKeyframe;
    lastKeyframe = lastKeyframePhase.lastKeyframe;
    lastMotion = lastKeyframePhase.currentMotion;
    if(currentKeyframeMotionRequest.keyframeMotion < KeyframeMotionID::firstNonGetUpAction)
    {
      forceBalancerInit = true;
      isMirror = lastKeyframePhase.isMirror;
    }
    else
      isMirror = currentKeyframeMotionRequest.mirror;
    // special case for demo wave
    if(lastKeyframePhase.currentKeyframeMotionRequest.keyframeMotion != KeyframeMotionID::demoBannerWave && lastKeyframePhase.currentKeyframeMotionRequest.keyframeMotion != KeyframeMotionID::demoBannerWaveInitial)
    {
      FOREACH_ENUM(Joints::Joint, joint)
      {
        lastJointRequest.angles[joint] = lastKeyframePhase.preHeatAdjustment.angles[joint];
        jointRequestOutput.angles[joint] = lastKeyframePhase.preHeatAdjustment.angles[joint];
      }
      engine.theEnergySaving.shutDown();
    }
    if(lastKeyframePhase.state == EngineState::breakUp)
    {
      lastMotion.reset();
      lastKeyframe = Keyframe();
      currentKeyframe = Keyframe();
    }
    else
      initStartJoints = false;
  }
  if(initStartJoints)
  {
    lastJointRequest = engine.theJointRequest;
    jointRequestOutput = engine.theJointRequest;
    lastUnbalanced = engine.theJointRequest;
  }
  forceStandMotionAfterPlaydead = lastPhase.type == MotionPhase::playDead;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    // replace with measurement
    if(lastJointRequest.angles[joint] == JointAngles::off)
      lastJointRequest.angles[joint] = jointRequestOutput.angles[joint] = lastUnbalanced.angles[joint] = engine.theJointAngles.angles[joint];
  }
  if(lastPhase.type != MotionPhase::keyframeMotion)
    engine.theEnergySaving.shutDown();

  // init ringbuffer
  for(std::size_t i = 0; i < lastTorsoAngle.capacity(); i++)
    lastTorsoAngle.push_front(Vector2a::Zero());

  // Init pastJointAngles
  for(std::size_t i = 0; i < pastJointAnglesAngles.capacity(); i++)
  {
    JointAngles lastRequest;
    lastRequest.angles = engine.theJointAngles.angles;
    pastJointAnglesAngles.push_front(lastRequest);
  }
}

void KeyframeMotionEngine::update(KeyframeMotionGenerator& output)
{
  setUpDebugCommands(output);

  output.createPhase = [this](const KeyframeMotionRequest& keyframeMotionRequest, const MotionPhase& lastPhase)
  {
    return std::make_unique<KeyframePhase>(*this, keyframeMotionRequest, lastPhase);
  };

  output.setPhaseInformation = [&output](const float ratio, const KeyframeMotionRequest::KeyframeMotionID motion)
  {
    output.ratio = ratio;
    output.motion = motion;
  };

  output.wasLastGetUp = [](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::getUp)
    {
      const auto& lastKeyframePhase = static_cast<const KeyframePhase&>(lastPhase);
      return lastKeyframePhase.currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::KeyframeMotionID::decideAutomatic;
    }
    return false;
  };
}

void KeyframeMotionEngine::update(GetUpGenerator& output)
{
  output.createPhase = [this](const MotionPhase& lastPhase)
  {
    KeyframeMotionRequest request;
    request.keyframeMotion = KeyframeMotionID::decideAutomatic;
    request.mirror = false;
    return std::make_unique<KeyframePhase>(*this, request, lastPhase);
  };
}

void KeyframeMotionEngine::update(DiveGenerator& output)
{
  output.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromDiveRequest(motionRequest.diveRequest);
    return std::make_unique<KeyframePhase>(*this, keyframeMotionRequest, lastPhase);
  };
}

void KeyframeMotionEngine::update(SpecialGenerator& output)
{
  output.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromSpecialRequest(motionRequest.specialRequest);
    return std::make_unique<KeyframePhase>(*this, keyframeMotionRequest, lastPhase);
  };
}

bool KeyframePhase::isDone(const MotionRequest& motionRequest) const
{
  const bool sitDownWaitForRequest = (currentKeyframeMotionRequest.keyframeMotion != KeyframeMotionID::sitDown || !engine.theGroundContactState.contact || motionRequest.motion != MotionRequest::playDead) && state == EngineState::waitForRequest;
  const bool isSitDownBreakUp = currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionID::sitDown && state == EngineState::breakUp;
  const auto isMotionCompatibleWithRequest = [this, &motionRequest]
  {
    switch(currentKeyframeMotionRequest.keyframeMotion)
    {
      case KeyframeMotionID::sitDownKeeper:
        return motionRequest.motion == MotionRequest::dive && motionRequest.diveRequest == MotionRequest::Dive::prepare;
      case KeyframeMotionID::keeperJumpLeft:
      case KeyframeMotionID::genuflectStand:
      case KeyframeMotionID::genuflectStandDefender:
        return motionRequest.motion == MotionRequest::dive;
      case KeyframeMotionID::demoBannerWave:
        return false; // This should run in a loop.
      case KeyframeMotionID::demoBannerWaveInitial:
        return motionRequest.motion == MotionRequest::special && motionRequest.specialRequest == MotionRequest::Special::demoBannerWaveInitial;
    }
    return false;
  };
  return ((sitDownWaitForRequest || isSitDownBreakUp) && !isMotionCompatibleWithRequest()) // (Motion is done OR sitDown special) cases AND request changed
         || (state == EngineState::balanceOut && motionRequest.isWalking() && type == MotionPhase::getUp && engine.theGroundContactState.contact) // Early switch to walkPhase
         || (state == EngineState::breakUp && engine.theFrameInfo.getTimeSince(breakUpTimestamp) > 1000 && type == MotionPhase::keyframeMotion && ((motionRequest.motion != MotionRequest::dive && motionRequest.motion != MotionRequest::special) || (engine.safeUprightParameters.pitchDirection.isInside(engine.theInertialData.angle.y()) && engine.safeUprightParameters.rollDirection.isInside(engine.theInertialData.angle.x()))))
         || (currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionID::sitDownKeeper && state == EngineState::working && (motionRequest.diveRequest == MotionRequest::Dive::jumpLeft || motionRequest.diveRequest == MotionRequest::Dive::jumpRight));
}

void KeyframePhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  lastJointRequest = jointRequestOutput;
  ASSERT(jointRequestOutput.isValid());
  jointRequest = jointRequestOutput;
  odometryOffset = odometry;
  //motionInfo.isMotionStable = engine.mofs[motionID].isMotionStable;
  motionInfo.executedKeyframeMotion.keyframeMotion = currentKeyframeMotionRequest.keyframeMotion;
  motionInfo.executedKeyframeMotion.mirror = isMirror;
  if(type == MotionPhase::getUp)
    motionInfo.getUpTryCounter = tryCounter;
}

std::unique_ptr<MotionPhase> KeyframePhase::createNextPhase(const MotionPhase& defaultPhase) const
{
  if(state == EngineState::breakUp && type == MotionPhase::keyframeMotion)
  {
    if(defaultPhase.type != MotionPhase::keyframeMotion && defaultPhase.type != MotionPhase::playDead)
    {
      KeyframeMotionRequest request;
      request.keyframeMotion = KeyframeMotionID::decideAutomatic;
      request.mirror = false;
      return std::make_unique<KeyframePhase>(engine, request, *this);
    }
  }
  if((currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionID::keeperJumpLeft ||
      currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionID::genuflectStand ||
      currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionID::genuflectStandDefender) &&
     defaultPhase.type != MotionPhase::playDead)
  {
    KeyframeMotionRequest request;
    request.keyframeMotion = KeyframeMotionID::decideAutomatic;
    request.mirror = false;
    return std::make_unique<KeyframePhase>(engine, request, *this);
  }
  else if(defaultPhase.type != MotionPhase::walk &&
          currentKeyframeMotionRequest.keyframeMotion == KeyframeMotionID::decideAutomatic)
    return engine.theWalkGenerator.createPhase(Pose2f(0, 0.1f, 0.f), *this, 0.f);
  return std::unique_ptr<MotionPhase>();
}

void KeyframePhase::update()
{
#ifndef NDEBUG
  DEBUG_RESPONSE_ONCE("module:KeyframeMotionEngine:toggleBalancer")
    balancerOn = !balancerOn;
#endif
  odometry = Pose2f();

  updateSensorArray();
  const JointAngles lastJointRequest = jointRequestOutput;
  jointRequestOutput.angles = preHeatAdjustment.angles;
  // check for special events
  doManipulateOwnState();

  // is this the first frame the KeyframeMotionEngine is on?
  if(state == EngineState::off)
  {
    // If there is a problem with the sensory update, wait until it is fixed (V6 head flex problem)
    if(engine.theGyroOffset.bodyDisconnect)
    {
      FOREACH_ENUM(Joints::Joint, joint)
        jointRequestOutput.angles[joint] = JointAngles::off;
      return;
    }
    // Init jointDiff. This is done to prevent wrong calculated joint differences in calculateJointDifference
    jointDiff1Angles.angles.fill(0);
    jointDiffPredictedAngles.angles.fill(0);
    lastNotActiveTimestamp = engine.theFrameInfo.time;
    state = EngineState::decideAction;
  }
  // Wait until we can start a new get up try.
  if(state == EngineState::breakUp)
    waitForFallen();
  // Wait for a requested motion
  if(state == EngineState::waitForRequest)
    // Get up finished but robot fell down again while waiting for a motion request change -> isInBreakUpRange changes the state and isDone() is false again
    static_cast<void>(isInBreakUpRange()); // no return, to allow energy saving at the end
  // Decide what to do
  if(state == EngineState::decideAction)
    // Check which motion shall be executed
    selectMotionToBeExecuted();
  // Check if we waited long enough and can continue our motion.
  if(state == EngineState::waiting)
  {
    const int waitTime = currentKeyframe.waitConditions.size() == 1 ? currentKeyframe.waitConditions[0].maxWaitTime : 1000;
    // Waiting over?
    if(checkEarlyBranch()
       || checkConditions(currentKeyframe.waitConditions, true)  // Condition is satisfied
       || engine.theFrameInfo.getTimeSince(waitTimestamp) >= waitTime // Wait time took to long
       || failedWaitCounter > 8.f) // Condition is not reachable
    {
      wasInWaiting = true;
      state = EngineState::working;
      initCurrentLine(true);
      updateLineValues();
    }
    // We are in break up range and need to break the motion?
    else if(!isInBreakUpRange())
    {
      // We are still in a waiting time. We set the next joints so we can still balance while we wait.
      ratio = 1.f;
      setNextJoints();
    }
  }
  // We are at the end of the get up motion and are standing. Keep balancing.
  if(state == EngineState::balanceOut)
    doBalanceOut();
  // Check if we can continue the current motion and set the next joints.
  if(state == EngineState::working)
  {
    // calc current ratio;
    isCurrentLineOver(); // if motion is over, state is set to finished
    // Need to break the motion?
    isInBreakUpRange(); // if in break up, state is set to break up and the robot needs to wait a moment
    // continue motion
    if(state == EngineState::working)
      setNextJoints();
  }
  if(state == EngineState::finished)
    checkFinishedState();
  // We have no get up tries left.
  if(state == EngineState::helpMeState)
    doHelpMeStuff();
  FOREACH_ENUM(Joints::Joint, joint)
    if(jointRequestOutput.stiffnessData.stiffnesses[joint] == 0)
      jointRequestOutput.angles[joint] = JointAngles::off;

  // apply heat adjustment
  preHeatAdjustment.angles = jointRequestOutput.angles;

  const bool energySavingLeg = (state == EngineState::waitForRequest && currentMotion.keyframeEndRequest.energySavingLegs == EnergySavingType::activeInWait) || currentMotion.keyframeEndRequest.energySavingLegs == EnergySavingType::activeAlways;
  const bool energySavingArms = (state == EngineState::waitForRequest && currentMotion.keyframeEndRequest.energySavingArms == EnergySavingType::activeInWait) || currentMotion.keyframeEndRequest.energySavingArms == EnergySavingType::activeAlways;
  if(engine.theEnergySaving.state == EnergySaving::EnergyState::resetState ||
     (energySavingLeg || energySavingArms))
  {
    engine.theEnergySaving.applyHeatAdjustment(jointRequestOutput, energySavingLeg, energySavingLeg, energySavingArms, energySavingArms,
                                               currentMotion.keyframeEndRequest.isStandHigh, false);
  }

  engine.theKeyframeMotionGenerator.setPhaseInformation(ratio, currentKeyframeMotionRequest.keyframeMotion);
}

void KeyframeMotionEngine::setUpDebugCommands(KeyframeMotionGenerator&)
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
  DECLARE_PLOT("module:KeyframeMotionEngine:ratio");
  DECLARE_DEBUG_RESPONSE("module:KeyframeMotionEngine:toggleBalancer");

  MODIFY_ONCE("module:KeyframeMotionEngine:setSteppingKeyframes", stepKeyframes);
  MODIFY("module:KeyframeMotionEngine:calculateDrawing", calculateDrawing);
#endif
}

MAKE_MODULE(KeyframeMotionEngine);
