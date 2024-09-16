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
#include "Debugging/Debugging.h"
#include "Math/BHMath.h"
#include "Math/Rotation.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Modules/Infrastructure/InterThreadProviders/PerceptionProviders.h"

MAKE_MODULE(MotionEngine);

MotionEngine::MotionEngine()
{
  playDeadGenerator.createPhase = [this](const MotionRequest&, const MotionPhase& lastPhase) -> std::unique_ptr<MotionPhase>
  {
    if(lastPhase.type != MotionPhase::playDead &&
       lastPhase.type != MotionPhase::fall &&
       !motionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDown) &&
       !motionInfo.isKeyframeMotion(KeyframeMotionRequest::keeperJumpLeft) &&
       !motionInfo.isKeyframeMotion(KeyframeMotionRequest::genuflectStand) &&
       !motionInfo.isKeyframeMotion(KeyframeMotionRequest::genuflectStandDefender))
    {
      KeyframeMotionRequest request;
      request.keyframeMotion = KeyframeMotionRequest::sitDown;
      request.mirror = false;
      return theKeyframeMotionGenerator.createPhase(request, lastPhase);
    }
    return std::make_unique<PlayDeadPhase>(lastPhase, *this);
  };

  generators[MotionRequest::playDead] = &playDeadGenerator;
  generators[MotionRequest::stand] = &theStandGenerator;
  generators[MotionRequest::walkAtAbsoluteSpeed] = &theWalkAtAbsoluteSpeedGenerator;
  generators[MotionRequest::walkAtRelativeSpeed] = &theWalkAtRelativeSpeedGenerator;
  generators[MotionRequest::walkToPose] = &theWalkToPoseGenerator;
  generators[MotionRequest::walkToBallAndKick] = &theWalkToBallAndKickGenerator;
  generators[MotionRequest::dribble] = &theDribbleGenerator;
  generators[MotionRequest::dive] = &theDiveGenerator;
  generators[MotionRequest::special] = &theSpecialGenerator;
  generators[MotionRequest::replayWalk] = &theReplayWalkRequestGenerator;
  generators[MotionRequest::photoMode] = &thePhotoModeGenerator;

  phase = std::make_unique<PlayDeadPhase>(*this);
}

void MotionEngine::update(JointRequest& jointRequest)
{
  ASSERT(phase);

  const JointRequest lastRequest = jointRequest;

  //1. update the current MotionPhase
  // Update the state of the currently active phase.
  phase->update();

  // Get oldest timestamp of lower, upper and cognition. If one thread stopped, the robot shall sit down
  const auto behaviorTimeStamps = { theCognitionFrameInfo.time, theUpperFrameInfo.time, theLowerFrameInfo.time };
  const unsigned int oldestBehaviorTimestamp = *std::min_element(std::begin(behaviorTimeStamps), std::end(behaviorTimeStamps));

  // Check if Cognition stopped or the IMU has an offset.
  if(oldestBehaviorTimestamp != lastCognitionTime && !theGyroOffset.isIMUBad && theGyroOffset.offsetCheckFinished)
    forceSitDown = false;
  else if(SystemCall::getMode() == SystemCall::physicalRobot &&
          !forceSitDown &&
          (theGyroOffset.isIMUBad || // Gyro has offsets
           (theFrameInfo.time > 110000 &&
            theFrameInfo.getTimeSince(oldestBehaviorTimestamp) > emergencySitDownDelay))) // No new camera images
  {
    forceSitDown = true;
    if(!theGyroOffset.isIMUBad)
    {
      OUTPUT_ERROR("No data from Cognition to Motion for more than " << ((emergencySitDownDelay + 500) / 1000) << " seconds.");
      SystemCall::playSound("sirene.wav", true);
      SystemCall::say("No cognition data!", true);
    }
    else
      OUTPUT_ERROR("Gyro values have high offsets!");
  }
  else if(!theGyroOffset.offsetCheckFinished)  // Gyro offsets could not be checked yet
    forceSitDown = true;
  lastCognitionTime = oldestBehaviorTimestamp;

  // Integrate special cases into the motion request.
  MotionRequest motionRequest = theMotionRequest;
  if(forceSitDown)
    motionRequest.motion = MotionRequest::playDead;

  // 2.1 check if a FallPhase should start ...
  const bool getUp = motionRequest.motion != MotionRequest::playDead && motionRequest.motion != MotionRequest::dive &&
                     (theFallDownState.state == FallDownState::fallen || theFallDownState.state == FallDownState::squatting) &&
                     !((phase->type == MotionPhase::getUp || phase->type == MotionPhase::stand) && theFallDownState.state == FallDownState::squatting); // If we got this combination, then the robot finished the get up and is hold tilted
  if(getUp)
  {
    motionRequest.motion = (phase->type == MotionPhase::playDead || phase->type == MotionPhase::keyframeMotion || phase->type == MotionPhase::getUp) ? MotionRequest::stand : MotionRequest::playDead;
    motionRequest.standHigh = false;
  }

  // Check if the fall engine should intervene (this can happen during phases).
  if(phase->type != MotionPhase::fall && theFallGenerator.shouldCatchFall(motionRequest))
    phase = theFallGenerator.createPhase();
  else if(phase->type != MotionPhase::freeze && theFreezeGenerator.shouldHandleBodyDisconnect(*phase))
    phase = theFreezeGenerator.createPhase();

  // 2.1 Check if the phase is done, i.e. a new phase has to be started.
  else if(phase->isDone(motionRequest))
  {
    // Update motion info.
    if(motionInfo.isKicking())
    {
      motionInfo.lastKickType = static_cast<KickInfo::KickType>(phase->kickType);
      motionInfo.lastKickTimestamp = theFrameInfo.time;
    }
    // Safe information about the start time
    motionInfo.lastMotionPhaseStarted = theFrameInfo.time;
    motionInfo.getUpTryCounter = 0;
    motionInfo.odometryAtLastPhaseSwitch = theOdometryDataPreview;
    motionInfo.odometryRequestAtLastPhaseSwitch = theOdometryTranslationRequest;

    // Check if we want to save the last phase
    theReplayWalkRequestGenerator.savePhase(*phase);
    std::unique_ptr<MotionPhase> newPhase = std::unique_ptr<MotionPhase>();

    // 2.2 create a new MotionPhase based on the MotionRequest
    // Create the next phase according to the request or get up if necessary.

    //should we intercept the ball?
    if(motionRequest.shouldInterceptBall && motionRequest.motion != MotionRequest::dive)
    {
      newPhase = theInterceptBallGenerator.createPhase(motionRequest, *phase);
    }
    // if the robot is in the getUp movement the movement has to be completed
    if(getUp)
    {
      // set get up motion as new motion phase
      newPhase = theGetUpGenerator.createPhase(*phase);
    }
    // if the  newPhase is a nullpointer e.g. no motionphase was set
    if(!newPhase)
    {
      newPhase = generators[motionRequest.motion]->createPhase(motionRequest, *phase); // no, execute MotionRequest
    }
    // should the robot intentionally walk out of the ball line?
    if(motionRequest.isWalking() && motionRequest.shouldWalkOutOfBallLine && !getUp && newPhase->type != MotionPhase::fall)
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
    if(auto nextPhase = phase->createNextPhase(*newPhase))
      phase = std::move(nextPhase);
    else
      phase = std::move(newPhase);

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
  MotionUtilities::copy(newJoints, jointRequest, theStiffnessSettings, static_cast<Joints::Joint>(0),
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
  for(const Joints::Joint joint : theDamageConfigurationBody.jointsToEraseStiffness)
    jointRequest.stiffnessData.stiffnesses[joint] = 0;

  for(const Joints::Joint joint : theJointPlay.getJointsWithSensorJump(JointPlay::broken))
  {
    if(brokenJointsReducesStiffnessList.end() != std::find(brokenJointsReducesStiffnessList.begin(), brokenJointsReducesStiffnessList.end(), joint))
      jointRequest.stiffnessData.stiffnesses[joint] = std::min(jointRequest.stiffnessData.stiffnesses[joint], brokenJointAutomaticStiffness);
  }

  // Let the request of turned off joints track the measurement.
  FOREACH_ENUM(Joints::Joint, joint)
    if(jointRequest.stiffnessData.stiffnesses[joint] == 0)
      jointRequest.angles[joint] = SystemCall::getMode() == SystemCall::simulatedRobot && lastRequest.angles[joint] != JointAngles::ignore && lastRequest.angles[joint] != JointAngles::off ? lastRequest.angles[joint] : theJointAngles.angles[joint];

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

std::unique_ptr<MotionPhase> PlayDeadPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  if(!(std::abs(engine.theInertialData.angle.y()) < engine.uprightAngle.y() && std::abs(engine.theInertialData.angle.x()) < engine.uprightAngle.x()))
  {
    if(engine.theFrameInfo.getTimeSince(uprightWarningTimestamp) > engine.uprightWarningTime)
      SystemCall::say("Please hold my upper! body! upright");
    return std::make_unique<PlayDeadPhase>(*this, engine);
  }

  if(defaultNextPhase.type == MotionPhase::stand)
    return std::unique_ptr<MotionPhase>();
  MotionRequest request = engine.theMotionRequest;
  request.motion = MotionRequest::stand;
  request.standHigh = false;
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
