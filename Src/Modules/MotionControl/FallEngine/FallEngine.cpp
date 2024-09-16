/*
 * @file FallEngine.cpp
 *
 * A minimized motion engine for falling.
 *
 * @author Bernd Poppinga
 */

#include "FallEngine.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Motion/MotionUtilities.h"
#include <cmath>

MAKE_MODULE(FallEngine);

void FallEngine::update(FallGenerator& fallGenerator)
{
  fallGenerator.shouldCatchFall = [this](const MotionRequest& motionRequest)
  {
    const bool isWavingRequest = motionRequest.motion == MotionRequest::special &&
                                 (motionRequest.specialRequest == MotionRequest::Special::demoBannerWave || motionRequest.specialRequest == MotionRequest::Special::demoBannerWaveInitial);
    const bool isWaving = theMotionInfo.executedPhase == MotionPhase::keyframeMotion &&
                          (theMotionInfo.executedKeyframeMotion.keyframeMotion == KeyframeMotionRequest::demoBannerWave ||
                           theMotionInfo.executedKeyframeMotion.keyframeMotion == KeyframeMotionRequest::demoBannerWaveInitial);
    return theFallDownState.state == FallDownState::falling &&
           motionRequest.motion != MotionRequest::dive &&
           (motionRequest.motion != MotionRequest::special || isWavingRequest) &&
           motionRequest.motion != MotionRequest::playDead &&
           (theMotionInfo.executedPhase != MotionPhase::keyframeMotion || isWaving) &&
           theMotionInfo.executedPhase != MotionPhase::getUp &&
           theMotionInfo.executedPhase != MotionPhase::playDead;
  };

  fallGenerator.createPhase = [this]
  {
    return std::make_unique<FallPhase>(*this);
  };
}

FallPhase::FallPhase(const FallEngine& engine) :
  MotionPhase(MotionPhase::fall),
  engine(engine),
  fallDirection(engine.theFallDownState.direction),
  startTime(engine.theFrameInfo.time)
{
  // Set stiffness to 10 for all joints (head will be overridden later).
  FOREACH_ENUM(Joints::Joint, i)
    request.stiffnessData.stiffnesses[i] = 10;

  // Start with the measured joint angles when the phase starts.
  request.angles = engine.theJointAngles.angles;
  mirrorFrontFall = engine.theInertialData.angle.x() < 0_deg;
}

void FallPhase::update()
{
  // As soon as the state has switched to fallen or some time has passed, a getUp request cancels the engine.
  if(engine.theFallDownState.state == FallDownState::fallen ||
     engine.theFrameInfo.getTimeSince(startTime) > engine.waitAfterFalling)
    waitingForGetUp = true;
  if(engine.theFallDownState.direction != fallDirection) // prevent oscillating
  {
    if((engine.theFallDownState.direction == FallDownState::back && engine.theInertialData.angle.y() < -engine.fallDirectionChangeThreshold) ||
       (engine.theFallDownState.direction == FallDownState::front && engine.theInertialData.angle.y() > engine.fallDirectionChangeThreshold) ||
       ((engine.theFallDownState.direction == FallDownState::left || engine.theFallDownState.direction == FallDownState::right) && std::abs(engine.theInertialData.angle.x()) > engine.fallDirectionChangeThreshold))
      fallDirection = engine.theFallDownState.direction;
  }
}

bool FallPhase::isDone(const MotionRequest& motionRequest) const
{
  // If the robot becomes upright / squatting on its own (or with human help) or a getUp is requested after the robot has reached the ground, the phase is done.
  return (engine.theFallDownState.state == FallDownState::upright ||
          engine.theFallDownState.state == FallDownState::squatting ||
          (waitingForGetUp && motionRequest.motion != MotionRequest::dive)) &&
         std::abs(engine.theInertialData.gyro.x()) < engine.maxGyroToStartGetUp &&
         std::abs(engine.theInertialData.gyro.y()) < engine.maxGyroToStartGetUp &&
         std::abs(engine.theInertialData.gyro.z()) < engine.maxGyroToStartGetUp;
}

void FallPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  safeBody(request);
  safeArms(request);

  setHeadStiffness(request);

  ASSERT(request.isValid());
  jointRequest = request;
  odometryOffset = Pose2f();
  motionInfo.isMotionStable = false;
}

std::unique_ptr<MotionPhase> FallPhase::createNextPhase(const MotionPhase& defaultPhase) const
{
  if(engine.theFallDownState.state == FallDownState::upright || engine.theFallDownState.state == FallDownState::staggering || defaultPhase.type == MotionPhase::playDead)
    return std::unique_ptr<MotionPhase>();
  return engine.theGetUpGenerator.createPhase(*this);
}

void FallPhase::setHeadStiffness(JointRequest& request)
{
  // Set head yaw stiffness high as long as it has not reached its target, then set it low.
  if(std::abs(engine.theJointAngles.angles[Joints::headYaw]) > 0.1f && !headYawInSafePosition)
    request.stiffnessData.stiffnesses[Joints::headYaw] = engine.stiffnessHead.max;
  else
  {
    headYawInSafePosition = true;
    request.stiffnessData.stiffnesses[Joints::headYaw] = engine.stiffnessHead.min;
  }

  // Set head pitch stiffness high as long as it has not reached its target, then set it low.
  if(std::abs(engine.theJointAngles.angles[Joints::headPitch] - request.angles[Joints::headPitch]) > 0.1f && !headPitchInSafePosition)
    request.stiffnessData.stiffnesses[Joints::headPitch] = engine.stiffnessHead.max;
  else
  {
    headPitchInSafePosition = true;
    request.stiffnessData.stiffnesses[Joints::headPitch] = engine.stiffnessHead.min;
  }
}

void FallPhase::safeBody(JointRequest& request)
{
  for(int i = 0; i < Joints::numOfJoints; i++)
    request.stiffnessData.stiffnesses[i] = 10;

  if(fallDirection == FallDownState::front)
  {
    JointAngles ref;
    if((engine.theInertialData.angle.y() > engine.lateFallYAngleFront && engine.theFrameInfo.getTimeSince(startTime) > engine.highStiffnessDuration) ||
       lateFallMotion)
    {
      lateFallMotion = true;
      ref.angles = engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitFrontAfterFall];
    }
    else
      ref.angles = engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitFront];

    if(mirrorFrontFall)
    {
      JointAngles anglesMirror;
      anglesMirror.mirror(ref);
      ref = anglesMirror;
    }
    MotionUtilities::copy(ref, request, Joints::firstLegJoint, Joints::numOfJoints);
  }
  else if(fallDirection == FallDownState::back)
  {
    if((engine.theInertialData.angle.y() < engine.lateFallYAngleBack && engine.theFrameInfo.getTimeSince(startTime) > engine.highStiffnessDuration) ||
       lateFallMotion)
    {
      lateFallMotion = true;
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitBackAfterFall],
                            request, Joints::firstLegJoint, Joints::numOfJoints);
    }
    else
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitBack],
                            request, Joints::firstLegJoint, Joints::numOfJoints);
  }
  else
  {
    MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sit],
                          request, Joints::firstLegJoint, Joints::numOfJoints);
  }
  const bool lowStiffness = engine.theFrameInfo.getTimeSince(startTime) > engine.highStiffnessDuration;
  const int useKneeStiffness = !lowStiffness || lateFallMotion ? engine.stiffnessLeg.max : engine.stiffnessLeg.min;
  const int useHipStiffness = !lowStiffness ? engine.stiffnessLeg.max : engine.stiffnessLeg.min;
  request.stiffnessData.stiffnesses[Joints::lKneePitch] = useKneeStiffness;
  request.stiffnessData.stiffnesses[Joints::rKneePitch] = useKneeStiffness;
  request.stiffnessData.stiffnesses[Joints::lHipPitch] = useHipStiffness;
  request.stiffnessData.stiffnesses[Joints::rHipPitch] = useHipStiffness;
}
void FallPhase::safeArms(JointRequest& request)
{
  const bool lowStiffness = engine.theFrameInfo.getTimeSince(startTime) > engine.highStiffnessDuration;
  const bool lowStiffnessElbow = engine.theFrameInfo.getTimeSince(startTime) > engine.highStiffnessDurationElbowRoll;
  // move arms behind, to damp the fall
  switch(fallDirection)
  {
    // move arms back and pull legs together
    case FallDownState::back:
    {
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitBack],
                            request, static_cast<Joints::Joint>(0), Joints::firstLegJoint);
      const int useStiffness = lowStiffness ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      const int useStiffnessElbow = lowStiffnessElbow ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      request.stiffnessData.stiffnesses[Joints::lShoulderRoll] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::lShoulderPitch] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::rShoulderRoll] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::rShoulderPitch] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::lElbowRoll] = useStiffnessElbow;
      request.stiffnessData.stiffnesses[Joints::rElbowRoll] = useStiffnessElbow;
      break;
    }
    // move arms to the front, to damp the fall
    case FallDownState::front:
    {
      JointAngles goal;
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sitFront],
                            goal, static_cast<Joints::Joint>(0), Joints::firstLegJoint);

      if(mirrorFrontFall)
      {
        JointAngles anglesMirror;
        anglesMirror.mirror(goal);
        goal = anglesMirror;
      }

      request.angles[Joints::lShoulderRoll] = goal.angles[Joints::lShoulderRoll];
      // make sure arm does not get stuck behind
      if(engine.theJointAngles.angles[Joints::lShoulderRoll] > 0_deg || leftShoulderRollLowStiffness)
      {
        request.angles[Joints::lShoulderPitch] = goal.angles[Joints::lShoulderPitch];
        request.angles[Joints::lElbowYaw] = goal.angles[Joints::lElbowYaw];
        request.angles[Joints::lElbowRoll] = goal.angles[Joints::lElbowRoll];
        request.angles[Joints::lWristYaw] = goal.angles[Joints::lWristYaw];
        leftShoulderRollLowStiffness = true;
      }
      else
      {
        // Boost the joints. No idea if this is still needed
        request.angles[Joints::lShoulderRoll] = 20_deg;
        request.angles[Joints::lElbowYaw] = -90_deg;
      }

      request.angles[Joints::rShoulderRoll] = goal.angles[Joints::rShoulderRoll];
      // make sure arm does not get stuck behind
      if(engine.theJointAngles.angles[Joints::rShoulderRoll] < 0_deg || rightShoulderRollLowStiffness)
      {
        request.angles[Joints::rShoulderPitch] = goal.angles[Joints::rShoulderPitch];
        request.angles[Joints::rElbowYaw] = goal.angles[Joints::rElbowYaw];
        request.angles[Joints::rElbowRoll] = goal.angles[Joints::rElbowRoll];
        request.angles[Joints::rWristYaw] = goal.angles[Joints::rWristYaw];
        rightShoulderRollLowStiffness = true;
      }
      else
      {
        // Boost the joints. No idea if this is still needed
        request.angles[Joints::rShoulderRoll] = -20_deg;
        request.angles[Joints::rElbowYaw] = -90_deg;
      }

      const int useStiffness = lowStiffness ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      request.stiffnessData.stiffnesses[Joints::rElbowYaw] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::rElbowYaw] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::lShoulderRoll] = leftShoulderRollLowStiffness || lowStiffness ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      request.stiffnessData.stiffnesses[Joints::lShoulderPitch] = useStiffness;
      request.stiffnessData.stiffnesses[Joints::rShoulderRoll] = rightShoulderRollLowStiffness || lowStiffness ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      request.stiffnessData.stiffnesses[Joints::rShoulderPitch] = useStiffness;

      request.angles[Joints::headPitch] = goal.angles[Joints::headPitch];
      request.angles[Joints::headYaw] = goal.angles[Joints::headYaw];
      break;
    }
    // pull legs together and wait until something happens
    default:
    {
      JointAngles goal;
      MotionUtilities::copy(engine.theStaticJointPoses.pose[StaticJointPoses::StaticJointPoseName::sit],
                            goal, static_cast<Joints::Joint>(0), Joints::firstLegJoint);

      const Angle shoulderPitchCorrection =
        Rangef::OneRange().limit(engine.theInertialData.angle.y() / engine.shoulderPitchSideFallCorrectionRange) * -engine.shoulderPitchSideFallCorrectionValue;

      request.angles[Joints::lShoulderRoll] = goal.angles[Joints::lShoulderRoll];
      if(engine.theJointAngles.angles[Joints::lShoulderRoll] > 10_deg || leftShoulderRollLowStiffness)
      {
        request.angles[Joints::lShoulderPitch] = goal.angles[Joints::lShoulderPitch];
        request.angles[Joints::lElbowYaw] = goal.angles[Joints::lElbowYaw];
        request.angles[Joints::lElbowRoll] = goal.angles[Joints::lElbowRoll];
        request.angles[Joints::lWristYaw] = goal.angles[Joints::lWristYaw];
        leftShoulderRollLowStiffness = true;
        if(fallDirection == FallDownState::right)
          request.angles[Joints::lShoulderPitch] += shoulderPitchCorrection;
      }
      else
      {
        // Boost the joints. No idea if this is still needed
        request.angles[Joints::lShoulderRoll] = 20_deg;
        request.angles[Joints::lElbowYaw] = 0_deg;
      }

      request.angles[Joints::rShoulderRoll] = goal.angles[Joints::rShoulderRoll];
      if(engine.theJointAngles.angles[Joints::rShoulderRoll] < -10_deg || rightShoulderRollLowStiffness)
      {
        request.angles[Joints::rShoulderPitch] = goal.angles[Joints::rShoulderPitch];
        request.angles[Joints::rElbowYaw] = goal.angles[Joints::rElbowYaw];
        request.angles[Joints::rElbowRoll] = goal.angles[Joints::rElbowRoll];
        request.angles[Joints::rWristYaw] = goal.angles[Joints::rWristYaw];
        rightShoulderRollLowStiffness = true;
        if(fallDirection == FallDownState::left)
          request.angles[Joints::rShoulderPitch] += shoulderPitchCorrection;
      }
      else
      {
        // Boost the joints. No idea if this is still needed
        request.angles[Joints::rShoulderRoll] = -20_deg;
        request.angles[Joints::rElbowYaw] = 0_deg;
      }

      request.stiffnessData.stiffnesses[Joints::lShoulderRoll] = leftShoulderRollLowStiffness || lowStiffness ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      request.stiffnessData.stiffnesses[Joints::lShoulderPitch] = engine.stiffnessArm.min;
      request.stiffnessData.stiffnesses[Joints::rShoulderRoll] = rightShoulderRollLowStiffness || lowStiffness ? engine.stiffnessArm.min : engine.stiffnessArm.max;
      request.stiffnessData.stiffnesses[Joints::rShoulderPitch] = engine.stiffnessArm.min;

      request.angles[Joints::headPitch] = goal.angles[Joints::headPitch];
      request.angles[Joints::headYaw] = goal.angles[Joints::headYaw];
    }
  }
}
