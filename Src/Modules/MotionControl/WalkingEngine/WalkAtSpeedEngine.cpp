/**
 * @file WalkAtSpeedEngine.cpp
 *
 * This file implements a module that provides walk generators.
 *
 * @author Arne Hasselbring
 */

#include "WalkAtSpeedEngine.h"
#include "Representations/MotionControl/MotionRequest.h"

MAKE_MODULE(WalkAtSpeedEngine, motionControl);

void WalkAtSpeedEngine::update(WalkAtAbsoluteSpeedGenerator& walkAtAbsoluteSpeedGenerator)
{
  walkAtAbsoluteSpeedGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const Pose2f requestSpeed = motionRequest.walkSpeed;
    const Vector2f useMaxSpeed(motionRequest.walkSpeed.translation.x() >= 0.f ? theWalkingEngineOutput.maxSpeed.translation.x() : std::abs(theWalkingEngineOutput.maxSpeedBackwards),
                               theWalkingEngineOutput.maxSpeed.translation.y());
    const Pose2f speedScaling(std::abs(requestSpeed.rotation) / theWalkingEngineOutput.maxSpeed.rotation, motionRequest.walkSpeed.translation.array() / useMaxSpeed.array());
    return createPhase(motionRequest.walkSpeed, speedScaling, lastPhase);
  };
}

void WalkAtSpeedEngine::update(WalkAtRelativeSpeedGenerator& walkAtRelativeSpeedGenerator)
{
  walkAtRelativeSpeedGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    return createPhase(Pose2f(theWalkingEngineOutput.maxSpeed.rotation,
                              motionRequest.walkSpeed.translation.x() > 0.f ? theWalkingEngineOutput.maxSpeed.translation.x() : std::abs(theWalkingEngineOutput.maxSpeedBackwards), theWalkingEngineOutput.maxSpeed.translation.y()),
                              motionRequest.walkSpeed, lastPhase);
  };
}

std::unique_ptr<MotionPhase> WalkAtSpeedEngine::createPhase(const Pose2f& walkTarget, const Pose2f& walkSpeed, const MotionPhase& lastPhase) const
{
  Pose2f walkSpeedScaled = walkSpeed;
  const float factor = std::abs(walkSpeedScaled.translation.x()) > 1.f || std::abs(walkSpeedScaled.translation.y()) > 1.f || std::abs(walkSpeedScaled.rotation) > 1.f ?
                       std::max(std::abs(walkSpeedScaled.rotation), std::max(std::abs(walkSpeedScaled.translation.x()), std::abs(walkSpeedScaled.translation.y()))) :
                       1.f;
  walkSpeedScaled.rotation /= factor;
  walkSpeedScaled.translation /= factor;

  const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(walkTarget.translation.y() != 0.f ? (walkTarget.translation.y() > 0.f) : (walkTarget.rotation > 0.f), lastPhase);
  Pose2f stepTarget = walkTarget;
  stepTarget.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeedScaled).clamped(walkTarget.rotation * walkSpeedScaled.rotation);

  std::vector<Vector2f> translationPolygon;
  theWalkGenerator.getTranslationPolygon(isLeftPhase, stepTarget.rotation, lastPhase, walkSpeedScaled, translationPolygon, false);

  Vector2f backRight(0.f, 0.f);
  Vector2f frontLeft(0.f, 0.f);
  for(const Vector2f& edge : translationPolygon)
  {
    backRight.x() = std::min(backRight.x(), edge.x());
    backRight.y() = std::min(backRight.y(), edge.y());
    frontLeft.x() = std::max(frontLeft.x(), edge.x());
    frontLeft.y() = std::max(frontLeft.y(), edge.y());
  }

  stepTarget.translation.x() = walkSpeedScaled.translation.x() >= 0.f ? frontLeft.x() : backRight.x();
  stepTarget.translation.y() = walkSpeedScaled.translation.y() >= 0.f ? frontLeft.y() : backRight.y();
  if(isLeftPhase == (stepTarget.translation.y() < 0.f))
    stepTarget.translation.y() = 0.f;
  return theWalkGenerator.createPhase(stepTarget, lastPhase);
}
