/**
 * @file WalkingEngine.cpp
 *
 * This file implements a module that is a wrapper for the UNSW walk generator.
 *
 * @author Thomas Röfer
 */

#include "WalkingEngine.h"

MAKE_MODULE(WalkingEngine, motionControl);

WalkingEngine::WalkingEngine()
{
  theWalkGenerator.reset();
}

void WalkingEngine::update(WalkingEngineOutput& walkingEngineOutput)
{
  beginFrame(theFrameInfo.time);
  execute(OptionInfos::getOption("Root"));
  endFrame();
}

void WalkingEngine::updateOutput(bool stepStarted, WalkingEngineOutput& walkingEngineOutput)
{
  if(stepStarted)
  {
    walkingEngineOutput.speed = theWalkGenerator.speed;
    walkingEngineOutput.upcomingOdometryOffset = theWalkGenerator.upcomingOdometryOffset;
    walkingEngineOutput.executedWalk = walkRequest;
    walkingEngineOutput.standing = walkingEngineOutput.speed == Pose2f()
                                   && walkingEngineOutput.odometryOffset == Pose2f();
  }

  walkingEngineOutput.isLeavingPossible = theFrameInfo.getTimeSince(lastTimeWalking) >= minTimeInStandBeforeLeaving;

  walkingEngineOutput.odometryOffset = theWalkGenerator.odometryOffset;
  walkingEngineOutput.upcomingOdometryOffset -= walkingEngineOutput.odometryOffset;
  walkingEngineOutput.maxSpeed = theWalkGenerator.maxSpeed;

  Pose2f odometryOffset = theWalkGenerator.odometryOffset;
  odometryOffset.rotation = Angle::normalize(Rotation::Euler::getZAngle(theInertialData.orientation3D) - theOdometryData.rotation);
  target -= odometryOffset;

  walkingEngineOutput.angles = jointRequest.angles;
  walkingEngineOutput.stiffnessData = jointRequest.stiffnessData;
}

void WalkingEngine::walk(const Pose2f& speed, WalkGenerator::WalkMode walkMode,
                         const std::function<Pose3f(float)>& getKickFootOffset)
{
  ASSERT(walkMode != WalkGenerator::targetMode);

  // Prevent standing
  this->speed = speed == Pose2f() ? Pose2f(0.000001f, 0.f) : speed;
  this->walkMode = walkMode;
  this->getKickFootOffset = getKickFootOffset;
  lastTarget = Pose2f(100000.f, 100000.f);
}

void WalkingEngine::walk(const Pose2f& speed, const Pose2f& target)
{
  // Prevent standing
  this->speed = speed;
  if(target != lastTarget)
    this->target = lastTarget = target;
  this->walkMode = WalkGenerator::targetMode;
  this->getKickFootOffset = std::function<Pose3f(float)>();
}

void WalkingEngine::stand()
{
  walkRequest = theMotionRequest.walkRequest;
  speed = Pose2f();
  this->walkMode = WalkGenerator::speedMode;
  this->getKickFootOffset = std::function<Pose3f(float)>();
  lastTarget = Pose2f(100000.f, 100000.f);
}

void WalkingEngine::updateWalkRequestWithoutKick()
{
  WalkRequest::WalkKickRequest walkKickRequest = walkRequest.walkKickRequest;
  walkRequest = theMotionRequest.walkRequest;
  walkRequest.walkKickRequest = walkKickRequest;
}
