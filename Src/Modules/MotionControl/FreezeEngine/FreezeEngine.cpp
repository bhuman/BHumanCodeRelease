/*
 * @file FreezeEngine.cpp
 *
 * Freeze the last motion for a second
 *
 * @author Philip Reichenberg
 */

#include "FreezeEngine.h"
#include "Platform/SystemCall.h"
#include <cmath>

MAKE_MODULE(FreezeEngine);

void FreezeEngine::update(FreezeGenerator& theFreezeGenerator)
{
  theFreezeGenerator.shouldHandleBodyDisconnect = [this](const MotionPhase& currentPhase)
  {
    return currentPhase.type != MotionPhase::playDead && theMotionRobotHealth.frameLostStatus == MotionRobotHealth::bodyDisconnect;
  };

  theFreezeGenerator.createPhase = [this]
  {
    return std::make_unique<FreezePhase>(*this);
  };
}

FreezePhase::FreezePhase(const FreezeEngine& engine) :
  MotionPhase(MotionPhase::freeze),
  engine(engine),
  reconnectTime(engine.theFrameInfo.time),
  startRequest(engine.theJointRequest)
{
  FOREACH_ENUM(Joints::Joint, i)
  {
    startRequest.stiffnessData.stiffnesses[i] = 60;
    if(startRequest.angles[i] == JointAngles::ignore || startRequest.angles[i] == JointAngles::off)
      startRequest.angles[i] = engine.theJointAngles.angles[i];
  }
}

void FreezePhase::update()
{
  startFallMotion = std::abs(engine.theInertialData.angle.y()) > engine.uprightBodyBoundary.y() ||
                    std::abs(engine.theInertialData.angle.x()) > engine.uprightBodyBoundary.x();
}

bool FreezePhase::isDone(const MotionRequest&) const
{
  return engine.theMotionRobotHealth.frameLostStatus != MotionRobotHealth::bodyDisconnect && (engine.theFrameInfo.getTimeSince(reconnectTime) > engine.freezeTime || startFallMotion);
}

void FreezePhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo&)
{
  if(startFallMotion)
  {
    FOREACH_ENUM(Joints::Joint, i)
      startRequest.stiffnessData.stiffnesses[i] = 30;
  }
  jointRequest = startRequest;
}

std::unique_ptr<MotionPhase> FreezePhase::createNextPhase(const MotionPhase&) const
{
  if(startFallMotion)
    return engine.theFallGenerator.createPhase();
  return std::unique_ptr<MotionPhase>();
}
