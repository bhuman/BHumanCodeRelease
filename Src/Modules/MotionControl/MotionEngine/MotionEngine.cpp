/**
 * @file MotionEngine.cpp
 *
 * This file implements a module that instructs other modules to create motions.
 *
 * @author Arne Hasselbring
 */

#include "MotionEngine.h"
#include "Debugging/Annotation.h"
#include "Debugging/Debugging.h"
#include "Framework/Settings.h"
#include "Math/BHMath.h"
#include "Math/Rotation.h"
#include "Modules/Infrastructure/InterThreadProviders/PerceptionProviders.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Streaming/Global.h"
#include "Tools/Motion/MotionUtilities.h"

MAKE_MODULE(MotionEngine);

MotionEngine::MotionEngine()
{
  playDeadGenerator.createPhase = [this](const MotionRequest&, const MotionPhase& lastPhase) -> std::unique_ptr<MotionPhase>
  {
    return std::make_unique<PlayDeadPhase>(lastPhase, *this);
  };

  generators[MotionRequest::playDead] = &playDeadGenerator;
  generators[MotionRequest::stand] = &theStandGenerator;
  generators[MotionRequest::walkAtAbsoluteSpeed] = &theWalkAtAbsoluteSpeedGenerator;
  generators[MotionRequest::walkAtRelativeSpeed] = &theWalkAtRelativeSpeedGenerator;
  generators[MotionRequest::walkToPose] = &theWalkToPoseGenerator;
  generators[MotionRequest::walkToBallAndKick] = &theWalkToBallAndKickGenerator;
  generators[MotionRequest::dribble] = &theDribbleGenerator;
  generators[MotionRequest::fallRequest] = &theFallGenerator;
  generators[MotionRequest::prepare] = &thePrepareGenerator;

  phase = std::make_unique<PlayDeadPhase>(*this);
}

void MotionEngine::update(JointRequest& jointRequest)
{
  ASSERT(phase);

  const JointRequest lastRequest = jointRequest;

  //1. update the current MotionPhase
  // Update the state of the currently active phase.
  phase->update();

  // Check if Cognition stopped or the IMU has an offset.
  if(theRobotHealth.olderPerceptionTimestamp != lastOlderPerceptionTimestamp && !theGyroOffset.isIMUBad && theGyroOffset.offsetCheckFinished)
    forceSitDown = false;
  else if(!forceSitDown &&
          (theGyroOffset.isIMUBad || // Gyro has offsets
           (theFrameInfo.time > 110000 &&
            theRobotHealth.olderPerceptionTimestamp &&
            theFrameInfo.getTimeSince(theRobotHealth.olderPerceptionTimestamp) > emergencySitDownDelay))) // No new camera images
  {
    forceSitDown = true;
    if(!theGyroOffset.isIMUBad)
    {
      OUTPUT_ERROR("No data from Cognition to Motion for more than " << ((emergencySitDownDelay + 500) / 1000) << " seconds.");
      SystemCall::playSound("siren.wav", true);
      SystemCall::say("No cognition dae ta", true);
    }
    else
      OUTPUT_ERROR("Gyro values have high offsets!");
  }
  else if(!theGyroOffset.offsetCheckFinished)  // Gyro offsets could not be checked yet
    forceSitDown = true;
  lastOlderPerceptionTimestamp = theRobotHealth.olderPerceptionTimestamp;

  // Integrate special cases into the motion request.
  MotionRequest motionRequest = theMotionRequest;
  if(forceSitDown)
  {
    // Booster robots can not just sitdown, theredown downgrade into the next safer motion
    switch(phase->type)
    {
      case MotionPhase::walk:
      case MotionPhase::stand:
      case MotionPhase::getUp:
        motionRequest.motion = MotionRequest::stand;
        break;
      default:
        motionRequest.motion = MotionRequest::playDead;
    }
  }

  // 2.1 check if a FallPhase should start ...
  const bool getUp = motionRequest.motion != MotionRequest::playDead &&
                     (theFallDownState.state == FallDownState::fallen && (phase->type == MotionPhase::fall || phase->type == MotionPhase::playDead));

  if(getUp)
  {
    motionRequest.motion = (phase->type == MotionPhase::fall || phase->type == MotionPhase::playDead || phase->type == MotionPhase::getUp) ? MotionRequest::stand : MotionRequest::playDead;
  }

  // Check if the fall engine should intervene (this can happen during phases).
  if(phase->type != MotionPhase::fall && theFallGenerator.shouldCatchFall(motionRequest))
    phase = theFallGenerator.createPhase(motionRequest, *phase);
  else if(phase->type != MotionPhase::playDead && theMotionRobotHealth.frameLostStatus == MotionRobotHealth::bodyDisconnect)
    phase = std::make_unique<PlayDeadPhase>(*this);

  // 2.1 Check if the phase is done, i.e. a new phase has to be started.
  else if(phase->isDone(motionRequest) || (motionRequest.motion == MotionRequest::playDead && forcePlayDead && phase->type != MotionPhase::playDead))
  {
    // Update motion info.
    if(motionInfo.isKicking() && phase->type == MotionPhase::walk)
    {
      motionInfo.lastKickType = static_cast<KickInfo::KickType>(phase->kickType);
      motionInfo.lastKickTimestamp = theFrameInfo.time;
      ASSERT(motionInfo.lastKickType < KickInfo::numOfKickTypes);
      if(motionInfo.lastKickType >= KickInfo::numOfKickTypes)
      {
        ANNOTATION("MotionEngine", "Invalid Kick Type");
        OUTPUT_ERROR("MotionEngine: Invalid Kick Type");
      }
    }
    else if(theBallKickInformation.lastBallContactTimestamp > motionInfo.lastKickTimestamp)
    {
      motionInfo.lastKickType = KickInfo::rlKickDynamic; // Workaround
      motionInfo.lastKickTimestamp = theBallKickInformation.lastBallContactTimestamp;
    }
    // Safe information about the start time
    motionInfo.lastMotionPhaseStarted = theFrameInfo.time;
    motionInfo.getUpTryCounter = 0;
    motionInfo.odometryAtLastPhaseSwitch = theOdometryDataPreview;

    std::unique_ptr<MotionPhase> newPhase = std::unique_ptr<MotionPhase>();

    // 2.2 create a new MotionPhase based on the MotionRequest
    // Create the next phase according to the request or get up if necessary.

    //should we intercept the ball?
    if(!forceSitDown && motionRequest.shouldInterceptBall)
    {
      newPhase = theInterceptBallGenerator.createPhase(motionRequest, *phase);
    }
    // if the robot is in the getUp movement the movement has to be completed
    if(getUp)
    {
      // set get up motion as new motion phase
      newPhase = theGetUpGenerator.createPhase(*phase);
    }
    // if the  newPhase is a nullpointer e.g. no motion phase was set
    if(!newPhase)
    {
      newPhase = generators[motionRequest.motion]->createPhase(motionRequest, *phase); // no, execute MotionRequest
    }
    // should the robot intentionally walk out of the ball line?
    if(!forceSitDown
       && (motionRequest.isWalking() || motionRequest.motion == MotionRequest::stand) // Walk or stand is requested
       && motionRequest.shouldWalkOutOfBallLine // walking out of ball is request
       && !getUp && newPhase->type != MotionPhase::fall // Robot is upright / not getting up. Fail-safe check
       && (phase->type == MotionPhase::stand || phase->type == MotionPhase::walk)) // Also only if walk engine was last active
    {
      // set new motion phase
      auto walkOutRequest = theWalkOutOfBallDirection.createPhase(motionRequest, *phase, *newPhase);
      // if the walkOutRequest is not a nullpointer
      if(walkOutRequest)
      {
        // walk intentionally out of the ball line
        newPhase = std::move(walkOutRequest);
      }
    }

    ASSERT(newPhase);

    // 2.3 create a new MotionPhase based on the previous one
    // Check if the previous phase has a mandatory continuation phase given the just created next phase.
    // 2.4 decide which one to use (follow up MotionPhases are prioritized)
    std::unique_ptr<MotionPhase> tempPhase;
    auto nextPhase = phase->createNextPhase(*newPhase);
    if(nextPhase && !(motionRequest.motion == MotionRequest::playDead && forcePlayDead))
      tempPhase = std::move(nextPhase);
    else
      tempPhase = std::move(newPhase);

    tempPhase->setLastPhase(phase);
    phase = std::move(tempPhase);

    // If walk phase then safe information about the swing foot
    if(phase->type == MotionPhase::walk)
      motionInfo.walkPhaseIsLeftPhase = theWalkGenerator.wasLastPhaseLeftPhase(*phase);
  }

  // The calcJoints methods should not write directly into the actual JointRequest,
  // otherwise accessing theJointRequest for the old angles will have unexpected behavior.
  JointRequest newJoints;
  newJoints.angles.fill(JointAngles::ignore);

  // Calculate limbs that can be controlled independently from the body.
  // 3. calculate head angles (if current MotionPhase allows it)
  const unsigned freeLimbs = phase->freeLimbs();
  theHeadMotionGenerator.calcJoints(freeLimbs & bit(MotionPhase::head), newJoints, headMotionInfo, motionRequest, theOdometryDataPreview);
  // 4. calculate the current arm joint angles (if current MotionPhase allows it)
  calcArmJoints(Arms::left, freeLimbs & bit(MotionPhase::leftArm), newJoints);
  calcArmJoints(Arms::right, freeLimbs & bit(MotionPhase::rightArm), newJoints);

  // Create the final joint request.
  Pose2f odometryOffset;
  if(phase->type == MotionPhase::stand && motionInfo.executedPhase != MotionPhase::stand)
    motionInfo.lastStandTimeStamp = theFrameInfo.time; // Set stand timeStamp
  motionInfo.executedPhase = phase->type;
  // 5. calculate the current(other) joint angles
  phase->calcJoints(motionRequest, newJoints, odometryOffset, motionInfo);

  if(phase->type != MotionPhase::walk)
    motionInfo.speed = Pose2f(0_deg, 0.f, 0.f);

  // Copy the new joint request into the final one (in order to convert ignore, off and useDefault).
  MotionUtilities::copy(newJoints, jointRequest, static_cast<Joints::Joint>(0),
                        static_cast<Joints::Joint>(Joints::numOfJoints));

  // 6. some extra handling
  // Check and clip joint angles.
#ifndef NDEBUG
  bool fail = false;
#endif
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(!std::isfinite(jointRequest.angles[joint]) || jointRequest.angles[joint] == JointAngles::ignore)
    {
#ifndef NDEBUG
      OUTPUT_ERROR("Joint " << TypeRegistry::getEnumName(joint) << " is invalid.");
      fail = true;
#endif
    }
    else if(theHeadMotionRequest.mode == HeadMotionRequest::calibrationMode && (joint == Joints::headPitch || joint == Joints::headYaw))
    {
      continue;
    }
    else if(!jointRequest.isTorqueControl)
      theJointLimits.limits[joint].clamp(jointRequest.angles[joint]);
  }
#ifndef NDEBUG
  if(fail | !jointRequest.stiffnessData.isValid(false))
  {
    OUTPUT_ERROR("Active phase: " << TypeRegistry::getEnumName(phase->type));
    FAIL("Some joints are invalid.");
  }
#endif

  // Let the request of turned off joints track the measurement.
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(jointRequest.stiffnessData.stiffnesses[joint] == 0)
      jointRequest.angles[joint] = SystemCall::getMode() == SystemCall::simulatedRobot && lastRequest.angles[joint] != JointAngles::ignore && lastRequest.angles[joint] != JointAngles::off ? lastRequest.angles[joint] : theJointAngles.angles[joint];
  }

  // Set this unused timestamp.
  jointRequest.timestamp = theFrameInfo.time;

  // Construct odometry update for this frame.
  odometryOffset.rotation = theOdometryDataPreview.odometryChange.rotation;
  odometryData = theOdometryDataPreview;
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
  return motionRequest.motion != MotionRequest::playDead &&
         (SystemCall::getMode() != SystemCall::physicalRobot
          || motionRequest.motion == MotionRequest::prepare);
}

void PlayDeadPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo& motionInfo)
{
  jointRequest.angles = engine.theJointAngles.angles;
  jointRequest.stiffnessData.stiffnesses.fill(0);
  motionInfo.isMotionStable = false;
}

std::unique_ptr<MotionPhase> PlayDeadPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  if(defaultNextPhase.type == MotionPhase::stand)
    return std::unique_ptr<MotionPhase>();
  MotionRequest request = engine.theMotionRequest;
  request.motion = SystemCall::getMode() == SystemCall::simulatedRobot ? MotionRequest::stand : MotionRequest::prepare;
  return engine.theStandGenerator.createPhase(request, *this);
}

void PlayDeadPhase::update()
{
  if(engine.theMotionRequest.motion == MotionRequest::playDead)
    uprightWarningTimestamp = engine.theFrameInfo.time;
}

PlayDeadPhase::PlayDeadPhase(const MotionPhase& lastPhase, const MotionEngine& engine) :
  MotionPhase(MotionPhase::playDead),
  engine(engine)
{
  if(lastPhase.type == MotionPhase::playDead)
  {
    const auto& lastPlayDeadPhase = static_cast<const PlayDeadPhase&>(lastPhase);
    uprightWarningTimestamp = engine.theFrameInfo.getTimeSince(lastPlayDeadPhase.uprightWarningTimestamp) > engine.uprightWarningTime ?
                              engine.theFrameInfo.time :
                              lastPlayDeadPhase.uprightWarningTimestamp;
  }
}
