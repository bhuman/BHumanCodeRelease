/*
 * @file RestrictiveFallEngine.cpp
 *
 * A minimized motion engine for falling.
 *
 * @author Philip Reichenberg
 */

#include "RestrictiveFallEngine.h"
#include "Framework/Settings.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Motion/MotionUtilities.h"
#include <cmath>

MAKE_MODULE(RestrictiveFallEngine);

void RestrictiveFallEngine::update(FallGenerator& fallGenerator)
{
  fallGenerator.shouldCatchFall = [this](const MotionRequest& motionRequest)
  {
    const bool isWavingRequest = motionRequest.motion == MotionRequest::special &&
                                 (motionRequest.specialRequest == MotionRequest::Special::demoBannerWave || motionRequest.specialRequest == MotionRequest::Special::demoBannerWaveInitial);
    const bool isWaving = theMotionInfo.executedPhase == MotionPhase::keyframeMotion &&
                          (theMotionInfo.executedKeyframeMotion.keyframeMotion == KeyframeMotionRequest::demoBannerWave ||
                           theMotionInfo.executedKeyframeMotion.keyframeMotion == KeyframeMotionRequest::demoBannerWaveInitial);
    return (std::abs(theInertialData.angle.y()) > fallAngle || std::abs(theInertialData.angle.x()) > fallAngle) &&
           motionRequest.motion != MotionRequest::dive &&
           (motionRequest.motion != MotionRequest::special || isWavingRequest) &&
           motionRequest.motion != MotionRequest::playDead &&
           (theMotionInfo.executedPhase != MotionPhase::keyframeMotion || isWaving) &&
           theMotionInfo.executedPhase != MotionPhase::getUp &&
           theMotionInfo.executedPhase != MotionPhase::playDead &&
           theGameState.playerState != GameState::PlayerState::penalizedManual;
  };

  fallGenerator.createPhase = [this]
  {
    return std::make_unique<RestrictiveFallPhase>(*this);
  };
}

RestrictiveFallPhase::RestrictiveFallPhase(const RestrictiveFallEngine& engine) :
  MotionPhase(MotionPhase::fall),
  engine(engine),
  startTime(engine.theFrameInfo.time)
{
  targetRequest.angles.fill(0_deg);
  lastRequest.angles = engine.theJointAngles.angles;
  VERIFY(InverseKinematic::calcLegJoints(Pose3f(Vector3f(0.f, engine.theWalkStepData.yHipOffset, -engine.standHeight)),
                                         Pose3f(Vector3f(0.f, -engine.theWalkStepData.yHipOffset, -engine.standHeight)),
                                         Vector2f(0.f, 0.f), targetRequest, engine.theRobotDimensions));

  lastRequest.stiffnessData.stiffnesses.fill(engine.stiffness);
  targetRequest.stiffnessData.stiffnesses.fill(engine.stiffness);
  targetRequest.angles[Joints::headPitch] = targetRequest.angles[Joints::headYaw] = 0_deg;
  targetRequest.angles[Joints::lShoulderPitch] = targetRequest.angles[Joints::rShoulderPitch] = 0_deg;
  targetRequest.angles[Joints::lShoulderRoll] = -83_deg;
  targetRequest.angles[Joints::lElbowYaw] = 90_deg;
  targetRequest.angles[Joints::lElbowRoll] = targetRequest.angles[Joints::rElbowRoll] = 0_deg;
  targetRequest.angles[Joints::rShoulderRoll] = 83_deg;
  targetRequest.angles[Joints::rElbowYaw] = -90_deg;
  startTime = engine.theFrameInfo.time;
  startTransitionTime = engine.theFrameInfo.time;
}

void RestrictiveFallPhase::update()
{
  bool transitionCouldStart = false;

  if(!(std::abs(engine.theInertialData.gyro.x()) < engine.maxGyroToStartGetUp &&  // gyros low
       std::abs(engine.theInertialData.gyro.y()) < engine.maxGyroToStartGetUp &&
       std::abs(engine.theInertialData.gyro.z()) < engine.maxGyroToStartGetUp &&
       (engine.allowGetUp || // any pose or upright
        (std::abs(engine.theInertialData.angle.x()) < engine.uprightThreshold &&
         std::abs(engine.theInertialData.angle.y()) < engine.uprightThreshold))))
    startTransitionTime = engine.theFrameInfo.time;
  else
    transitionCouldStart = true;

  if((engine.theFrameInfo.getTimeSince(startTime) > engine.waitAfterFalling) &&
     engine.theFrameInfo.getTimeSince(startTransitionTime) > engine.transitionDelay)
    allowLeavingFallPhase = true;

  if((engine.theFrameInfo.getTimeSince(startTime) > engine.waitAfterFalling) && transitionCouldStart
     && engine.theFrameInfo.getTimeSince(lastSoundTimestamp) > engine.soundTimeDelay)
  {
    lastSoundTimestamp = engine.theFrameInfo.time;
    SystemCall::say("Starting Get Up");
  }
}

bool RestrictiveFallPhase::isDone(const MotionRequest& motionRequest) const
{
  // If the robot becomes upright / squatting on its own (or with human help) or a getUp is requested after the robot has reached the ground, the phase is done.
  return allowLeavingFallPhase && motionRequest.motion != MotionRequest::dive;
}

void RestrictiveFallPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  const Angle jointSpeed = engine.jointSpeed * Global::getSettings().motionCycleTime;
  const Rangea speedLimit(-jointSpeed, jointSpeed);
  const Rangea positionLimit(-engine.maxPositionDifference, engine.maxPositionDifference);
  FOREACH_ENUM(Joints::Joint, joint)
  {
    const Angle maxTargetPosition = positionLimit.limit(targetRequest.angles[joint] - lastRequest.angles[joint]) + lastRequest.angles[joint];
    lastRequest.angles[joint] += speedLimit.limit(maxTargetPosition - lastRequest.angles[joint]);
  }
  ASSERT(lastRequest.isValid());

  if(lastSoundTimestamp != 0)
    FOREACH_ENUM(Joints::Joint, joint)
      lastRequest.stiffnessData.stiffnesses[joint] = std::min(100, lastRequest.stiffnessData.stiffnesses[joint] + 1);

  jointRequest = lastRequest;
  odometryOffset = Pose2f();
  motionInfo.isMotionStable = false;
}

std::unique_ptr<MotionPhase> RestrictiveFallPhase::createNextPhase(const MotionPhase& defaultPhase) const
{
  if((std::abs(engine.theInertialData.angle.x()) < engine.uprightThreshold &&
      std::abs(engine.theInertialData.angle.y()) < engine.uprightThreshold) || defaultPhase.type == MotionPhase::playDead)
    return std::unique_ptr<MotionPhase>();
  return engine.theGetUpGenerator.createPhase(*this);
}
