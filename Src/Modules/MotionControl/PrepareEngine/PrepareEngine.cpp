/**
 * @file PrepareEngine.cpp
 *
 * @author Philip Reichenberg
 */

#include "PrepareEngine.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(PrepareEngine);

PrepareEngine::PrepareEngine()
{
  target.angles[Joints::lShoulderRoll] = shoulderRoll;
  target.angles[Joints::rShoulderRoll] = -shoulderRoll;
  target.angles[Joints::lElbowPitch] = elbowYaw;
  target.angles[Joints::rElbowPitch] = elbowYaw;
}

void PrepareEngine::update(PrepareGenerator& thePrepareGenerator)
{
  thePrepareGenerator.createPhase = [this](const MotionRequest&, const MotionPhase&)
  {
    return std::make_unique<PreparePhase>(*this);
  };
}

PreparePhase::PreparePhase(PrepareEngine& engine) :
  MotionPhase(MotionPhase::prepare),
  engine(engine)
{
  start = engine.theJointAngles;
  phaseStart = engine.theFrameInfo.time;
}

void PreparePhase::update()
{
}

bool PreparePhase::isDone(const MotionRequest& motionRequest) const
{
  return motionRequest.motion != MotionRequest::prepare;
}

void PreparePhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo&)
{
  if(engine.theFrameInfo.time == phaseStart)
    SystemCall::say("Prepare", true);
  const float stiffnessTorsoRatio = mapToRange(std::max(std::abs(engine.theInertialData.angle.x()), std::abs(engine.theInertialData.angle.y())), static_cast<float>(engine.stiffnessTorsoRange.min), static_cast<float>(engine.stiffnessTorsoRange.max), 1.f, 0.f);
  const float ratio = Rangef::ZeroOneRange().limit(engine.theFrameInfo.getTimeSince(phaseStart) / engine.interpolationTime);
  engine.target.angles[Joints::headPitch] = mapToRange(stiffnessTorsoRatio, 0.f, 1.f, (engine.theInertialData.angle.y() > 0.f ? -1.f : 1.f) * static_cast<float>(engine.headFallAngle), 0.f);
  FOREACH_ENUM(Joints::Joint, joint)
    jointRequest.angles[joint] = engine.target.angles[joint] * ratio + start.angles[joint] * (1.f - ratio);

  jointRequest.stiffnessData.stiffnesses.fill(static_cast<int>(mapToRange(stiffnessTorsoRatio, 0.f, 1.f, engine.stiffnessRange.min, engine.stiffnessRange.max)));

  if(engine.theFrameInfo.getTimeSince(phaseStart) > engine.energySaving)
  {
    FOREACH_ENUM(Joints::Joint, joint)
      jointRequest.angles[joint] = Rangea(engine.theJointAngles.angles[joint] - engine.maxJointDiff, engine.theJointAngles.angles[joint] + engine.maxJointDiff).limit(engine.target.angles[joint]);
  }
}

std::unique_ptr<MotionPhase> PreparePhase::createNextPhase(const MotionPhase& nextPhase) const
{
  if(nextPhase.type != MotionPhase::playDead)
    SystemCall::say("Walk", true);
  return std::unique_ptr<MotionPhase>();
}
