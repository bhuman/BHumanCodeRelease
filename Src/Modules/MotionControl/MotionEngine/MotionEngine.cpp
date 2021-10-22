/**
 * @file MotionEngine.cpp
 *
 * This file implements a module that instructs other modules to create motions.
 *
 * @author Arne Hasselbring
 */

#include "MotionEngine.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Motion/MotionUtilities.h"

MAKE_MODULE(MotionEngine, motionControl);

MotionEngine::MotionEngine()
{
  playDeadGenerator.createPhase = [this](const MotionRequest&, const MotionPhase&)
  {
    return std::make_unique<PlayDeadPhase>(*this);
  };

  generators[MotionRequest::playDead] = &playDeadGenerator;
  generators[MotionRequest::stand] = &theStandGenerator;
  generators[MotionRequest::walkAtAbsoluteSpeed] = &theWalkAtAbsoluteSpeedGenerator;
  generators[MotionRequest::walkAtRelativeSpeed] = &theWalkAtRelativeSpeedGenerator;
  generators[MotionRequest::walkToPose] = &theWalkToPoseGenerator;
  generators[MotionRequest::walkToBallAndKick] = &theWalkToBallAndKickGenerator;
  generators[MotionRequest::dribble] = &theDribbleGenerator;
  generators[MotionRequest::getUp] = &theGetUpGenerator;
  generators[MotionRequest::keyframeMotion] = &theKeyframeMotionGenerator;
  generators[MotionRequest::replayWalk] = &theReplayWalkRequestGenerator;

  phase = std::make_unique<PlayDeadPhase>(*this);
}

void MotionEngine::update(JointRequest& jointRequest)
{
  ASSERT(phase);

  // Update the state of the currently active phase.
  phase->update();

  // Check if Cognition stopped.
  if(theCognitionFrameInfo.time != lastCognitionTime)
  {
    forceSitDown = false;
    lastCognitionTime = theCognitionFrameInfo.time;
  }
  else if(SystemCall::getMode() == SystemCall::physicalRobot &&
          !forceSitDown &&
          theCognitionFrameInfo.time &&
          theFrameInfo.time > 120000 &&
          theFrameInfo.getTimeSince(theCognitionFrameInfo.time) > emergencySitDownDelay)
  {
    forceSitDown = true;
    OUTPUT_ERROR("No data from Cognition to Motion for more than " << ((emergencySitDownDelay + 500) / 1000) << " seconds.");
  }

  // Integrate special cases into the motion request.
  MotionRequest motionRequest = theMotionRequest;
  if(forceSitDown && (motionInfo.isMotion(bit(MotionPhase::stand) | bit(MotionPhase::walk) | bit(MotionPhase::kick) | bit(MotionPhase::getUp)) || motionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDown)))
  {
    motionRequest.motion = MotionRequest::keyframeMotion;
    motionRequest.keyframeMotionRequest.keyframeMotion = KeyframeMotionRequest::sitDown;
    motionRequest.keyframeMotionRequest.mirror = false;
  }

  // Check if the fall engine should intervene (this can happen during phases).
  if(phase->type != MotionPhase::fall && theFallGenerator.shouldCatchFall(motionRequest))
    phase = theFallGenerator.createPhase();
  // Check if the phase is done, i.e. a new phase has to be started.
  else if(phase->isDone(motionRequest))
  {
    // Update motion info.
    if(motionInfo.isKicking())
    {
      motionInfo.lastKickType = static_cast<KickInfo::KickType>(phase->kickType);
      motionInfo.lastKickTimestamp = theFrameInfo.time;
    }
    motionInfo.getUpTryCounter = 0;

    // Check if we want to save the last phase
    theReplayWalkRequestGenerator.savePhase(*phase);

    // Create the next phase according to the request.
    auto newPhase = generators[motionRequest.motion]->createPhase(motionRequest, *phase);
    ASSERT(newPhase);

    // Check if the previous phase has a mandatory continuation phase given the just created next phase.
    if(auto nextPhase = phase->createNextPhase(*newPhase))
      phase = std::move(nextPhase);
    else
      phase = std::move(newPhase);
  }

  // The calcJoints methods should not write directly into the actual JointRequest,
  // otherwise accessing theJointRequest for the old angles will have unexpected behavior.
  JointRequest newJoints;
  newJoints.angles.fill(JointAngles::ignore);

  // Calculate limbs that can be controlled independently from the body.
  const unsigned freeLimbs = phase->freeLimbs();
  theHeadMotionGenerator.calcJoints(freeLimbs & bit(MotionPhase::head), newJoints, headMotionInfo);
  calcArmJoints(Arms::left, freeLimbs & bit(MotionPhase::leftArm), newJoints);
  calcArmJoints(Arms::right, freeLimbs & bit(MotionPhase::rightArm), newJoints);

  // Create the final joint request.
  Pose2f odometryOffset;
  motionInfo.executedPhase = phase->type;
  phase->calcJoints(motionRequest, newJoints, odometryOffset, motionInfo);

  if(phase->type != MotionPhase::walk)
    motionInfo.speed = Pose2f(0_deg, 0.f, 0.f);

  // Copy the new joint request into the final one (in order to convert ignore, off and useDefault).
  MotionUtilities::copy(newJoints, jointRequest, theStiffnessSettings, static_cast<Joints::Joint>(0),
                        static_cast<Joints::Joint>(Joints::numOfJoints));

  // Check and clip joint angles.
  bool fail = false;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(!std::isfinite(jointRequest.angles[joint]) || jointRequest.angles[joint] == JointAngles::ignore)
    {
#ifndef NDEBUG
      OUTPUT_ERROR("Joint " << TypeRegistry::getEnumName(joint) << " is invalid.");
      fail = true;
#endif
    }
    else if(theHeadAngleRequest.disableClippingAndInterpolation && (joint == Joints::headPitch || joint == Joints::headYaw))
    {
      continue;
    }
    else
      theJointLimits.limits[joint].clamp(jointRequest.angles[joint]);
  }
#ifndef NDEBUG
  if(fail | !jointRequest.stiffnessData.isValid(false))
  {
    OUTPUT_ERROR("Active phase: " << TypeRegistry::getEnumName(phase->type));
    FAIL("Some joints are invalid.");
  }
#endif

  // Set stiffness of broken joints to 0.
  for(Joints::Joint joint : theDamageConfigurationBody.jointsToEraseStiffness)
    jointRequest.stiffnessData.stiffnesses[joint] = 0;

  // Let the request of turned off joints track the measurement.
  FOREACH_ENUM(Joints::Joint, joint)
    if(jointRequest.stiffnessData.stiffnesses[joint] == 0)
      jointRequest.angles[joint] = theJointAngles.angles[joint];

  // Set this unused timestamp.
  jointRequest.timestamp = theFrameInfo.time;

  // Construct odometry update for this frame.
  if(theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen)
    odometryOffset.rotation = 0_deg; // Postpone rotation change until being upright again to avoid huge variances in the self locator.
  else
    odometryOffset.rotation = Angle::normalize(Rotation::Euler::getZAngle(theInertialData.orientation3D) - odometryData.rotation);
  odometryData += odometryOffset;
}

void MotionEngine::calcArmJoints(Arms::Arm arm, bool setJoints, JointRequest& jointRequest)
{
  if(!setJoints)
  {
    armMotionInfo.armMotion[arm] = ArmMotionInfo::none;
    return;
  }

  ArmMotionRequest armMotionRequest = theArmMotionRequest;
  if(theArmMotionRequest.armMotion[arm] == ArmMotionRequest::none)
  {
    if(armMotionInfo.armMotion[arm] == ArmMotionRequest::keyFrame && !armMotionInfo.isFree[arm])
    {
      armMotionRequest.armMotion[arm] = ArmMotionRequest::keyFrame;
      armMotionRequest.armKeyFrameRequest.arms[arm].motion = ArmKeyFrameRequest::reverse;
    }
  }

  switch(armMotionRequest.armMotion[arm])
  {
    case ArmMotionRequest::keyFrame:
      theArmKeyFrameGenerator.calcJoints(arm, armMotionRequest, jointRequest, armMotionInfo);
      break;
    case ArmMotionRequest::pointAt:
      thePointAtGenerator.calcJoints(arm, armMotionRequest, jointRequest, armMotionInfo);
      break;
    default:
      armMotionInfo.armMotion[arm] = ArmMotionInfo::none;
      break;
  }
}

bool PlayDeadPhase::isDone(const MotionRequest& motionRequest) const
{
  return motionRequest.motion != MotionRequest::playDead;
}

void PlayDeadPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo& motionInfo)
{
  jointRequest.angles = engine.theJointAngles.angles;
  jointRequest.stiffnessData.stiffnesses.fill(0);
  motionInfo.isMotionStable = false;
}
