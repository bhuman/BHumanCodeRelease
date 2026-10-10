/**
 * @file GetUpEngine.h
 *
 * @Author Philip Reichenberg
 */

#include "GetUpEngine.h"
#include "Debugging/Plot.h"
#include "Math/Rotation.h"
#include "Platform/SystemCall.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Modeling/BallPhysics.h"
#include <filesystem>

MAKE_MODULE(GetUpEngine);

GetUpEngine::GetUpEngine() :
  policy(&Global::getAsmjitRuntime())
{
  compile(false);
  // https://github.com/BoosterRobotics/booster_gym/blob/main/deploy/configs/T1.yaml#L19
  offset.angles[Joints::lHipPitch] = -0.2f;
  offset.angles[Joints::lKneePitch] = 0.4f;
  offset.angles[Joints::lAnklePitch] = -0.25f;
  offset.angles[Joints::rHipPitch] = -0.2f;
  offset.angles[Joints::rKneePitch] = 0.4f;
  offset.angles[Joints::rAnklePitch] = -0.25f;

  offsetFast.angles[Joints::lShoulderRoll] = -1.4f;
  offsetFast.angles[Joints::lElbowPitch] = -0.4f;
  offsetFast.angles[Joints::rShoulderRoll] = 1.4f;
  offsetFast.angles[Joints::rElbowPitch] = 0.4f;
  offsetFast.angles[Joints::lHipPitch] = -0.2f;
  offsetFast.angles[Joints::lKneePitch] = 0.4f;
  offsetFast.angles[Joints::lAnklePitch] = -0.2f;
  offsetFast.angles[Joints::rHipPitch] = -0.2f;
  offsetFast.angles[Joints::rKneePitch] = 0.4f;
  offsetFast.angles[Joints::rAnklePitch] = -0.2f;

  std::vector<Joints::Joint> upperBodyJoints = { Joints::headYaw,
                                                 Joints::headPitch,
                                                 Joints::lShoulderPitch,
                                                 Joints::lShoulderRoll,
                                                 Joints::lElbowPitch,
                                                 Joints::lElbowYaw,
                                                 Joints::rShoulderPitch,
                                                 Joints::rShoulderRoll,
                                                 Joints::rElbowPitch,
                                                 Joints::rElbowYaw,
                                               };

  std::vector<Joints::Joint> lowerBodyJoints = { Joints::lHipPitch,
                                                 Joints::lHipRoll,
                                                 Joints::lHipYaw,
                                                 Joints::lKneePitch,
                                                 Joints::lAnklePitch,
                                                 Joints::lAnkleRoll,
                                                 Joints::rHipPitch,
                                                 Joints::rHipRoll,
                                                 Joints::rHipYaw,
                                                 Joints::rKneePitch,
                                                 Joints::rAnklePitch,
                                                 Joints::rAnkleRoll
                                               };

  boosterWaistJoints = upperBodyJoints;
  boosterJoints = upperBodyJoints;

  boosterWaistJoints.push_back(Joints::waistYaw);

  boosterWaistJoints.insert(boosterWaistJoints.end(), lowerBodyJoints.begin(), lowerBodyJoints.end());
  boosterJoints.insert(boosterJoints.end(), lowerBodyJoints.begin(), lowerBodyJoints.end());

  recoveryPose.angles.fill(0);
  recoveryPose.angles[Joints::lShoulderRoll] = -80_deg;
  recoveryPose.angles[Joints::rShoulderRoll] = 80_deg;

  fallAngles.angles.fill(0);
  fallAngles.angles[Joints::headPitch] = fallAngles.angles[Joints::headYaw] = 0_deg;
  fallAngles.angles[Joints::lShoulderPitch] = fallAngles.angles[Joints::rShoulderPitch] = 0_deg;
  fallAngles.angles[Joints::lShoulderRoll] = -83_deg;
  fallAngles.angles[Joints::lElbowPitch] = 90_deg;
  fallAngles.angles[Joints::lElbowYaw] = fallAngles.angles[Joints::rElbowYaw] = 0_deg;
  fallAngles.angles[Joints::rShoulderRoll] = 83_deg;
  fallAngles.angles[Joints::rElbowPitch] = 90_deg;

  if(!fastActive)
    maxTryCounter++;

  if(SystemCall::getMode() == SystemCall::simulatedRobot)
    fastActive = false;
}

void GetUpEngine::compile(bool output)
{
  if(output)
  {
    if(!std::filesystem::exists(modelPath + policyName))
    {
      OUTPUT_ERROR("File " << modelPath << policyName << " does not exist");
      return;
    }
  }
  else
    ASSERT(std::filesystem::exists(modelPath + policyName));

  policy.compile(Model(modelPath + policyName));
  ASSERT(policy.valid());

  ASSERT(policy.numOfInputs() == 1);
  ASSERT(policy.input(0).rank() == 1);
  ASSERT(policy.input(0).dims(0) == numInput);

  ASSERT(policy.numOfOutputs() == 1);
  ASSERT(policy.output(0).rank() == 1);
  ASSERT(policy.output(0).dims(0) == numOutput);

  ASSERT(std::filesystem::exists(modelPath + policyNameFast));
  policyFast.compile(Model(modelPath + policyNameFast));
  ASSERT(policyFast.valid());

  ASSERT(policyFast.numOfInputs() == 1);
  ASSERT(policyFast.input(0).rank() == 1);
  ASSERT(policyFast.input(0).dims(0) == numInputFast);

  ASSERT(policyFast.numOfOutputs() == 1);
  ASSERT(policyFast.output(0).rank() == 1);
  ASSERT(policyFast.output(0).dims(0) == numOutputFast);
}

void GetUpEngine::update(GetUpGenerator& theGetUpGenerator)
{
  bool calcVelocity = true;
  if(lastFrameInfo == 0 || lastFrameInfo > theFrameInfo.time)
  {
    lastFrameInfo = theFrameInfo.time;
    calcVelocity = false;
  }
  const float numberFrames = std::max(1.f, std::floor((theFrameInfo.time - lastFrameInfo) / 2.f));
  if(numberFrames > 2.5f)  // Booster robots sometimes have longer data drops. In that case it is better to use boosters velocity value
    calcVelocity = false;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    lastMeasurement.velocity[joint] = !calcVelocity ? static_cast<float>(theJointAngles.velocity[joint]) : (theJointAngles.angles[joint] - lastMeasurement.angles[joint]) * 500.f / numberFrames;
  }
  lastFrameInfo = theFrameInfo.time;
  lastMeasurement.angles = theJointAngles.angles;

  theGetUpGenerator.createPhase = [this](const MotionPhase&)->std::unique_ptr<MotionPhase>
  {
    return std::make_unique<GetUpPhase>(*this);
  };
}

GetUpPhase::GetUpPhase(GetUpEngine& engine) :
  MotionPhase(MotionPhase::getUp),
  engine(engine)
{
  setUpRecovery();
  if(!engine.fastActive)
    tryCounter++;
  else
  {
    startTime = engine.theFrameInfo.time;
    startAngles = engine.theJointAngles;
    state = State::standUp;
    recoverMotionIndex = 0;
    executedTime = 0.f;
    isFront = engine.theInertialData.angle.y() > 0_deg;
    maxStandUpTime = 1500;
  }
}

void GetUpPhase::executePolicy(JointAngles& target)
{
  if(tryCounter > 0)
  {
    // Policy learned with 50 hz -> every 20 ms inference
    if(engine.theFrameInfo.getTimeSince(lastInference) < 20)
      return;

    if(maxStandUpTime == 0.f)
    {
      ASSERT(maxStandUpTime > 0.f);
      target.angles = engine.theJointAngles.angles;
      return;
    }

    JointAngles clippedJointAngles = engine.theJointAngles;
    FOREACH_ENUM(Joints::Joint, joint)
      clippedJointAngles.angles[joint] = engine.theJointLimits.limits[joint].limit(clippedJointAngles.angles[joint]);

    float* input = engine.policy.input(0).data();

    const Vector3f gravity = engine.theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);

    // gravity
    *input++ = gravity.x();
    *input++ = gravity.y();
    *input++ = gravity.z();

    // angular momentum
    *input++ = engine.theInertialData.gyro.x();
    *input++ = engine.theInertialData.gyro.y();
    *input++ = engine.theInertialData.gyro.z();

    // phase
    *input++ = Rangef::ZeroOneRange().limit(executedTime / maxStandUpTime); // value between 0 (start) and 1 (end)

    // joint sequence
    std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();

    // Measurements
    for(Joints::Joint j : jointList)
      *input++ = clippedJointAngles.angles[j] - engine.offset.angles[j];

    // Velocities
    for(Joints::Joint j : jointList)
      *input++ = engine.lastMeasurement.velocity[j] * 0.1f;

    // Last Requests
    for(Joints::Joint j : jointList)
      *input++ = engine.theJointRequest.angles[j] - engine.offset.angles[j];

    // Run network.
    STOPWATCH("module:GetUpEngine:apply")
      engine.policy.apply();

    // Get next request
    const float* output = engine.policy.output(0).data();
    for(Joints::Joint j : jointList)
      target.angles[j] = engine.clipActions.limit(*output++) + engine.offset.angles[j];
  }
  else
  {
    // Policy learned with 50 hz -> every 20 ms inference
    if(engine.theFrameInfo.getTimeSince(lastInference) < 20)
      return;

    JointAngles clippedJointAngles = engine.theJointAngles;
    FOREACH_ENUM(Joints::Joint, joint)
      clippedJointAngles.angles[joint] = engine.theJointLimits.limits[joint].limit(clippedJointAngles.angles[joint]);

    float* input = engine.policyFast.input(0).data();

    const Vector3f gravity = engine.theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);

    // angular momentum
    *input++ = engine.theInertialData.gyro.x();
    *input++ = engine.theInertialData.gyro.y();
    *input++ = engine.theInertialData.gyro.z();
    // gravity
    *input++ = gravity.x();
    *input++ = gravity.y();
    *input++ = gravity.z();

    // joint sequence
    std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();

    // Measurements
    for(Joints::Joint j : jointList)
      *input++ = clippedJointAngles.angles[j] - engine.offsetFast.angles[j];

    // Velocities
    for(Joints::Joint j : jointList)
      *input++ = engine.lastMeasurement.velocity[j] * 0.1f;

    // Last Requests
    for(Joints::Joint j : jointList)
      *input++ = lastAction.angles[j];

    // Run network.
    STOPWATCH("module:GetUpEngineFast:apply")
      engine.policyFast.apply();

    // Get next request
    const float* output = engine.policyFast.output(0).data();
    for(Joints::Joint j : jointList)
    {
      lastAction.angles[j] = engine.clipActions.limit(*output++);
      target.angles[j] = lastAction.angles[j] + clippedJointAngles.angles[j];
    }

    lastInference = engine.theFrameInfo.time;
  }

  lastInference = engine.theFrameInfo.time;
}

void GetUpPhase::update()
{
  if(shouldBreakUp())
  {
    state = State::breakUp;
    startTime = engine.theFrameInfo.time;
    tryCounter += SystemCall::getMode() == SystemCall::simulatedRobot ? 0 : 1;
  }

  if(tryCounter >= engine.maxTryCounter || (state == State::breakUp && engine.theFrameInfo.getTimeSince(engine.theRobotHealth.olderPerceptionTimestamp) > 1000))
    state = State::helpMe;

  switch(state)
  {
    case State::recovery:
    {
      ASSERT(recoverMotion);

      if(recoverMotion && engine.theFrameInfo.getTimeSince(startTime) >= (*recoverMotion)[recoverMotionIndex].duration)
      {
        recoverMotionIndex++;
        if(recoverMotionIndex < recoverMotion->size())   // keep executing recover motion
        {
          startAngles.angles = engine.theJointRequest.angles;
        }
        else if(engine.theGameState.isPenalized() && engine.theGameState.gameControllerActive && SystemCall::getMode() != SystemCall::simulatedRobot)
          state = State::helpMe;
        else // Start get up
        {
          state = State::standUp;
          isFront = engine.theInertialData.angle.y() > 0;
          maxStandUpTime = tryCounter == 0 ? 1500 : (isFront ? engine.frontInfo.back().executionTime : engine.backInfo.back().executionTime);
        }
        executedTime = 0.f;
        startTime = engine.theFrameInfo.time;
      }
      break;
    }
    case State::breakUp:
    {
      if(engine.theFrameInfo.getTimeSince(startTime) >= engine.breakUpTime   // Waited long enough after break up
         && tryCounter < engine.maxTryCounter) // We still have at least one try left
      {
        if(!engine.theGameState.stopped)  // The game is currently NOT stopped
          setUpRecovery();
        else if(engine.theFrameInfo.getTimeSince(lastStopSoundTimestamp) > engine.stopSoundTime)
        {
          lastStopSoundTimestamp = engine.theFrameInfo.time;
          SystemCall::say("Stand Up Paused", true);
        }
      }
      break;
    }
    case State::standUp:
    {
      executedTime = engine.theFrameInfo.getTimeSince(startTime) * engine.speedFactor;
      const float headHeight = (engine.theTorsoMatrix * engine.theRobotDimensions.hipToNeckOffset).z();

      if(tryCounter > 0
         && ((executedTime / maxStandUpTime >= 1.f
              || (executedTime / maxStandUpTime >= engine.earliestDoneTime
                  && engine.theTorsoMatrix.translation.z() > engine.minStandHeightWhenDone && headHeight > engine.minStandHeightWhenDone + engine.theRobotDimensions.hipToNeckOffset.z()))
             && (engine.theFallDownState.state == FallDownState::upright || engine.theFallDownState.state == FallDownState::staggering)))
        state = State::done;

      else if(tryCounter == 0 &&
              (engine.theTorsoMatrix.translation.z() > engine.minStandHeightWhenDone && headHeight > engine.minStandHeightWhenDone + engine.theRobotDimensions.hipToNeckOffset.z()
               && (engine.theFallDownState.state == FallDownState::upright || engine.theFallDownState.state == FallDownState::staggering)))
        state = State::done;
      break;
    }
    case State::helpMe:
    {
      if(!engine.theGameState.isPenalized() && tryCounter < engine.maxTryCounter)
        setUpRecovery();
      else if(engine.theFrameInfo.getTimeSince(lastHelpMeSound) > engine.helpMeSoundTimeWindow)
      {
        lastHelpMeSound = engine.theFrameInfo.time;
        SystemCall::say("Help me");
        SystemCall::playSound("mimimi.wav");
      }
      break;
    }
  }
}

bool GetUpPhase::isDone(const MotionRequest& request) const
{
  return request.motion == MotionRequest::playDead
         || request.motion == MotionRequest::prepare
         || state == State::done;
}

void GetUpPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  switch(state)
  {
    case State::recovery:
      doRecovery(nextRequest);
      break;
    case State::standUp:
      executePolicy(nextRequest);
      nextRequest.stiffnessData.stiffnesses.fill(100);
      if(tryCounter > 0)
      {
        nextRequest.stiffnessData.stiffnesses.fill(100);
        nextRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = engine.anklePitchStiffness;
        nextRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = engine.anklePitchStiffness;
      }
      break;
    case State::breakUp:
    case State::helpMe:
      doBreakUp(nextRequest);
      break;
  }

  jointRequest = nextRequest;
  if(tryCounter == 0)
  {
    jointRequest.isTorqueControl = true;
    jointRequest.jointStiffnessMapping = JointStiffnessMapping::Type::low;
  }

  // else set head, and keep everything default, so walk can overwrite it

  motionInfo.isMotionStable = false;
  motionInfo.isWalkPhaseInWalkKick = false;
  motionInfo.speed = Pose2f();

  odometryOffset = Pose2f(0_deg, 0.f, 0.f);
}

void GetUpPhase::doRecovery(JointRequest& request)
{
  ASSERT(recoverMotion);
  const float ratio = std::min(engine.theFrameInfo.getTimeSince(startTime) / (*recoverMotion)[recoverMotionIndex].duration, 1.f);
  FOREACH_ENUM(Joints::Joint, joint)
    request.angles[joint] = startAngles.angles[joint] * (1.f - ratio) + (*recoverMotion)[recoverMotionIndex].positions[joint] * ratio;
  request.stiffnessData.stiffnesses.fill(100);
}

void GetUpPhase::getRefTorso(Rangea& torsoXRange, Rangea& torsoYRange)
{
  if(tryCounter > 0)
  {
    ASSERT(state != State::recovery);
    ASSERT(state != State::breakUp);
    ASSERT(engine.frontInfo.size() > 1);
    ASSERT(engine.backInfo.size() > 1);
    const float executedRatio = executedTime / engine.recoveryTime;
    const auto& refInfo = isFront ? engine.frontInfo : engine.backInfo;

    std::size_t startIndex = 0;
    std::size_t nextIndex = 0;
    for(std::size_t index = 0; index < refInfo.size(); index++)
    {
      const float nextMaxRatio = refInfo[index].executionTime / engine.recoveryTime;
      if(nextMaxRatio < executedRatio)
        startIndex = index;
      else
      {
        nextIndex = index;
        break;
      }
    }

    const float refStartTime = refInfo[startIndex].executionTime;
    const float refNextTime = refInfo[nextIndex].executionTime;
    const float torsoRatio = Rangef::ZeroOneRange().limit((executedTime - refStartTime) / (refNextTime - refStartTime));
    torsoXRange.min = refInfo[startIndex].torsoXRange.min * (1.f - torsoRatio) + refInfo[nextIndex].torsoXRange.min * torsoRatio;
    torsoXRange.max = refInfo[startIndex].torsoXRange.max * (1.f - torsoRatio) + refInfo[nextIndex].torsoXRange.max * torsoRatio;
    torsoYRange.min = refInfo[startIndex].torsoYRange.min * (1.f - torsoRatio) + refInfo[nextIndex].torsoYRange.min * torsoRatio;
    torsoYRange.max = refInfo[startIndex].torsoYRange.max * (1.f - torsoRatio) + refInfo[nextIndex].torsoYRange.max * torsoRatio;
  }
  else
  {
    minTorsoAngle.x() = std::min(-20_deg, std::min(minTorsoAngle.x(), engine.theInertialData.angle.x()));
    minTorsoAngle.y() = std::min(-20_deg, std::min(minTorsoAngle.y(), engine.theInertialData.angle.y()));
    maxTorsoAngle.x() = std::max(20_deg, std::max(maxTorsoAngle.x(), engine.theInertialData.angle.x()));
    maxTorsoAngle.y() = std::max(20_deg, std::max(maxTorsoAngle.y(), engine.theInertialData.angle.y()));

    const bool lastAllowBreakUp = allowBreakUp;
    allowBreakUp = allowBreakUp || (isFront
                                    ? maxTorsoAngle.y() - engine.theInertialData.angle.y() > engine.maxTorsoAngleDiff
                                    : minTorsoAngle.y() - engine.theInertialData.angle.y() < -engine.maxTorsoAngleDiff);

    if(allowBreakUp)
    {
      if(!lastAllowBreakUp)
      {
        closestToZeroAngle.x() = engine.theInertialData.angle.x();
        closestToZeroAngle.y() = engine.theInertialData.angle.y();
      }
      else
      {
        closestToZeroAngle.x() = std::min(std::abs(engine.theInertialData.angle.x()), std::abs(closestToZeroAngle.x())) * (engine.theInertialData.angle.x() > 0_deg ? 1.f : -1.f);

        closestToZeroAngle.y() = isFront
                                 ? std::max(0_deg, std::min(engine.theInertialData.angle.y(), closestToZeroAngle.y()))
                                 : std::min(0_deg, std::max(engine.theInertialData.angle.y(), closestToZeroAngle.y()));
      }

      torsoXRange = Rangea(closestToZeroAngle.x() - engine.maxTorsoAngleDiff, closestToZeroAngle.x() + engine.maxTorsoAngleDiff);
      torsoYRange = Rangea(closestToZeroAngle.y() - engine.maxTorsoAngleDiff, closestToZeroAngle.y() + engine.maxTorsoAngleDiff);
    }
    else
    {
      torsoXRange = Range(-180_deg, 180_deg);
      torsoYRange = Range(-180_deg, 180_deg);
    }
  }
}

bool GetUpPhase::shouldBreakUp()
{
  if(tryCounter > 0)
  {
    if(state == State::standUp && executedTime > (isFront ? engine.frontInfo : engine.backInfo)[0].executionTime)
    {
      Rangea torsoXRange;
      Rangea torsoYRange;
      getRefTorso(torsoXRange, torsoYRange);
      return !torsoXRange.isInside(engine.theInertialData.angle.x()) || !torsoYRange.isInside(engine.theInertialData.angle.y());
    }
    return false;
  }
  else
  {
    if(state == State::standUp)
    {
      Rangea torsoXRange;
      Rangea torsoYRange;
      getRefTorso(torsoXRange, torsoYRange);
      const bool headHeight = (engine.theTorsoMatrix * engine.theRobotDimensions.hipToNeckOffset).z() < engine.minStandHeightWhenDone;
      return !torsoXRange.isInside(engine.theInertialData.angle.x()) || !torsoYRange.isInside(engine.theInertialData.angle.y())
             || (executedTime > maxStandUpTime && (std::abs(engine.theInertialData.angle.x()) > 45_deg || std::abs(engine.theInertialData.angle.x()) > 45_deg) && headHeight)
             || executedTime > maxStandUpTime * 2.0;
    }
    return false;
  }
}

void GetUpPhase::doBreakUp(JointRequest& request)
{
  const Angle jointSpeed = engine.breakUpJointSpeed * Global::getSettings().motionCycleTime;
  const Rangea speedLimit(-jointSpeed, jointSpeed);
  JointAngles targetAngles = engine.fallAngles;
  targetAngles.angles[Joints::headPitch] = (engine.theInertialData.angle.y() > 0.f ? -1.f : 1.f) * engine.breakUpHeadAngle;
  request.stiffnessData.stiffnesses.fill(0);
  request.angles = engine.theJointAngles.angles;
}

std::unique_ptr<MotionPhase> GetUpPhase::createNextPhase(const MotionPhase& nextPhase) const
{
  if(nextPhase.type == MotionPhase::prepare || nextPhase.type == MotionPhase::playDead)
    return std::unique_ptr<MotionPhase>();
  // Force walk afterwards
  return engine.theWalkGenerator.createPhase(Pose2f(0.f, 0.01f, 0.f), *this, 0.f);
}

std::vector<Joints::Joint> GetUpPhase::getBoosterLegJointSequence()
{
  if(engine.useWaist)
    return engine.boosterWaistJoints;
  return engine.boosterJoints;
}

void GetUpPhase::setUpRecovery()
{
  startTime = engine.theFrameInfo.time;
  startAngles = engine.theJointAngles;

  state = State::recovery;
  if(std::abs(engine.theInertialData.angle.x()) >= 40_deg && engine.theInertialData.angle.y() < 30_deg)
    recoverMotion = &engine.recoverBack;
  else if(std::abs(engine.theInertialData.angle.x()) >= 40_deg)
    recoverMotion = &engine.recoverFront;
  else
    recoverMotion = &engine.recoverNormal;
  recoverMotionIndex = 0;
  executedTime = 0.f;

  minTorsoAngle = Vector2a::Zero();
  maxTorsoAngle = Vector2a::Zero();
  closestToZeroAngle = Vector2a::Zero();
  allowBreakUp = false;
}
