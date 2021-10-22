/*
 * @file FallEngine.cpp
 *
 * A minimized motion engine for falling.
 *
 * @author Bernd Poppinga
 */

#include "FallEngine.h"
#include "Platform/SystemCall.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Motion/MotionUtilities.h"
#include <cmath>

MAKE_MODULE(FallEngine, motionControl);

void FallEngine::update(FallGenerator& fallGenerator)
{
  fallGenerator.shouldCatchFall = [this](const MotionRequest& motionRequest)
  {
    return theFallDownState.state == FallDownState::falling &&
           motionRequest.motion != MotionRequest::keyframeMotion &&
           motionRequest.motion != MotionRequest::getUp &&
           motionRequest.motion != MotionRequest::playDead &&
           theMotionInfo.executedPhase != MotionPhase::keyframeMotion &&
           theMotionInfo.executedPhase != MotionPhase::getUp &&
           theMotionInfo.executedPhase != MotionPhase::playDead;
  };

  fallGenerator.createPhase = [this]()
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
  return engine.theFallDownState.state == FallDownState::upright ||
         engine.theFallDownState.state == FallDownState::squatting ||
         (waitingForGetUp && motionRequest.motion == MotionRequest::getUp &&
          std::abs(engine.theInertialData.gyro.x()) < engine.maxGyroToStartGetUp &&
          std::abs(engine.theInertialData.gyro.y()) < engine.maxGyroToStartGetUp &&
          std::abs(engine.theInertialData.gyro.z()) < engine.maxGyroToStartGetUp);
}

void FallPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  safeBody(request);
  safeArms(request);

  // Pitch the head such that it points away from the floor.
  request.angles[Joints::headYaw] = 0;
  request.angles[Joints::headPitch] = 0;
  if(fallDirection == FallDownState::front)
    request.angles[Joints::headPitch] = -23_deg;
  else if(fallDirection == FallDownState::back)
    request.angles[Joints::headPitch] = 20_deg;
  else // fall direction unknown
    request.angles[Joints::headPitch] = 0_deg;

  // Set head yaw stiffness high as long as it has not reached its target, then set it low.
  if(std::abs(engine.theJointAngles.angles[Joints::headYaw]) > 0.1f && !headYawInSafePosition)
    request.stiffnessData.stiffnesses[Joints::headYaw] = 50;
  else
  {
    headYawInSafePosition = true;
    request.stiffnessData.stiffnesses[Joints::headYaw] = 20;
  }

  // Set head pitch stiffness high as long as it has not reached its target, then set it low.
  if(std::abs(engine.theJointAngles.angles[Joints::headPitch] - request.angles[Joints::headPitch]) > 0.1f && !headPitchInSafePosition)
    request.stiffnessData.stiffnesses[Joints::headPitch] = 50;
  else
  {
    headPitchInSafePosition = true;
    request.stiffnessData.stiffnesses[Joints::headPitch] = 20;
  }

  ASSERT(request.isValid());
  jointRequest = request;
  odometryOffset = Pose2f();
  motionInfo.isMotionStable = false;
}

void FallPhase::safeBody(JointRequest& request)
{
  for(int i = 0; i < Joints::numOfJoints; i++)
    request.stiffnessData.stiffnesses[i] = 10;

  if(fallDirection == FallDownState::front)
    MotionUtilities::sitFront(request);
  else
    MotionUtilities::sit(request);
  if(engine.theJointAngles.angles[Joints::lKneePitch] < 100_deg)
    request.stiffnessData.stiffnesses[Joints::lKneePitch] = 20;
  if(engine.theJointAngles.angles[Joints::rKneePitch] < 100_deg)
    request.stiffnessData.stiffnesses[Joints::rKneePitch] = 20;
}
void FallPhase::safeArms(JointRequest& request)
{
  if(engine.theFrameInfo.getTimeSince(startTime) > 5000)
  {
    JointRequest armRequest;
    MotionUtilities::stand(armRequest);
    MotionUtilities::copy(armRequest, request, engine.theStiffnessSettings, Joints::firstArmJoint, Joints::firstLegJoint);
    for(size_t i = Joints::firstArmJoint; i < Joints::firstLegJoint; i++)
      request.stiffnessData.stiffnesses[i] = SystemCall::getMode() == SystemCall::physicalRobot ? 0 : 1;
  }
  // move arms behind, to damp the fall
  else if(fallDirection == FallDownState::back)
  {
    MotionUtilities::safeArmsBehind(request);
    leftShoulderPitchLowStiffness = true;
    leftShoulderRollLowStiffness = true;
    rightShoulderPitchLowStiffness = true;
    rightShoulderRollLowStiffness = true;
  }
  // move arms to the front, to damp the fall
  else if(fallDirection == FallDownState::front)
  {
    JointRequest goal;
    MotionUtilities::safeArmsFront(goal);
    request.angles[Joints::lShoulderRoll] = goal.angles[Joints::lShoulderRoll];
    // make sure arm does not get stuck behind
    if(engine.theJointAngles.angles[Joints::lShoulderRoll] > 0_deg)
    {
      request.angles[Joints::lShoulderPitch] = goal.angles[Joints::lShoulderPitch];
      request.angles[Joints::lElbowYaw] = goal.angles[Joints::lElbowYaw];
      request.angles[Joints::lElbowRoll] = goal.angles[Joints::lElbowRoll];
      request.angles[Joints::lWristYaw] = goal.angles[Joints::lWristYaw];
      leftShoulderRollLowStiffness = true;
    }
    if(engine.theJointAngles.angles[Joints::lShoulderPitch] < 50_deg)
      leftShoulderPitchLowStiffness = true;

    request.angles[Joints::rShoulderRoll] = goal.angles[Joints::rShoulderRoll];
    // make sure arm does not get stuck behind
    if(engine.theJointAngles.angles[Joints::rShoulderRoll] < 5_deg)
    {
      request.angles[Joints::rShoulderPitch] = goal.angles[Joints::rShoulderPitch];
      request.angles[Joints::rElbowYaw] = goal.angles[Joints::rElbowYaw];
      request.angles[Joints::rElbowRoll] = goal.angles[Joints::rElbowRoll];
      request.angles[Joints::rWristYaw] = goal.angles[Joints::rWristYaw];
      rightShoulderRollLowStiffness = true;
    }
    if(engine.theJointAngles.angles[Joints::rShoulderPitch] < 80_deg)
      rightShoulderPitchLowStiffness = true;

    request.stiffnessData.stiffnesses[Joints::lShoulderRoll] = leftShoulderRollLowStiffness ? 10 : 50;
    request.stiffnessData.stiffnesses[Joints::lShoulderPitch] = leftShoulderPitchLowStiffness ? 10 : 20;
    request.stiffnessData.stiffnesses[Joints::rShoulderRoll] = rightShoulderRollLowStiffness ? 10 : 50;
    request.stiffnessData.stiffnesses[Joints::rShoulderPitch] = rightShoulderPitchLowStiffness ? 10 : 20;
  }
  else
  {
    leftShoulderPitchLowStiffness = true;
    rightShoulderPitchLowStiffness = true;
    request.angles[Joints::lShoulderRoll] = 20_deg;
    if(engine.theJointAngles.angles[Joints::lShoulderRoll] > 10_deg)
    {
      request.angles[Joints::lShoulderPitch] = 77_deg;
      request.angles[Joints::lElbowYaw] = 0_deg;
      request.angles[Joints::lElbowRoll] = -15_deg;
      request.angles[Joints::lWristYaw] = -90_deg;
      leftShoulderRollLowStiffness = true;
    }
    request.angles[Joints::rShoulderRoll] = -20_deg;
    if(engine.theJointAngles.angles[Joints::rShoulderRoll] < -10_deg)
    {
      request.angles[Joints::rShoulderPitch] = 77_deg;
      request.angles[Joints::rElbowYaw] = 0_deg;
      request.angles[Joints::rElbowRoll] = 15_deg;
      request.angles[Joints::rWristYaw] = 90_deg;
      rightShoulderRollLowStiffness = true;
    }
    request.stiffnessData.stiffnesses[Joints::lShoulderRoll] = leftShoulderRollLowStiffness ? 10 : 30;
    request.stiffnessData.stiffnesses[Joints::lShoulderPitch] = 10;
    request.stiffnessData.stiffnesses[Joints::rShoulderRoll] = rightShoulderRollLowStiffness ? 10 : 30;
    request.stiffnessData.stiffnesses[Joints::rShoulderPitch] = 10;
  }
}
