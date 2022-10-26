/**
 * @file WalkGenerator.h
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Math/Pose2f.h"
#include "Math/Pose3f.h"
#include "Tools/Motion/MotionPhase.h"
#include "Math/Range.h"
#include "Tools/Motion/WalkKickStep.h"
#include <memory>

STREAMABLE(WalkGenerator,
{
  using CreateNextPhaseCallback = std::function<std::unique_ptr<MotionPhase>(const MotionPhase&)>;

  FUNCTION(bool(const bool shouldBeLeftPhase, const MotionPhase& lastPhase)) isNextLeftPhase;
  FUNCTION(float(const Pose2f& velocity)) calcNextStepDuration;
  FUNCTION(Rangea(const bool isLeftPhase, const Pose2f& walkSpeedRatio)) getRotationRange;
  FUNCTION(void(const bool isLeftPhase, const float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio,
                std::vector<Vector2f>& translationPolygon, std::vector<Vector2f>& translationPolygonNoCenter, const bool fastWalk)) getTranslationPolygon;

  FUNCTION(std::unique_ptr<MotionPhase>(const Pose2f& step, const MotionPhase& lastPhase)) createPhase;
  FUNCTION(std::unique_ptr<MotionPhase>(const WalkKickStep& walkKickStep, const MotionPhase& lastPhase, const CreateNextPhaseCallback& createNextPhaseCallback)) createPhaseWithNextPhase;

  FUNCTION(std::tuple<Pose2f, Pose2f, Pose2f>(const MotionPhase& lastPhase)) getStartOffsetOfNextWalkPhase;
  FUNCTION(Pose2f(const MotionPhase& lastPhase)) getLastStepChange;
  FUNCTION(bool(const MotionPhase& lastPhase)) wasLastPhaseLeftPhase;
  FUNCTION(bool(const MotionPhase& lastPhase)) wasLastPhaseInWalkKick;

  FUNCTION(void(WalkKickStep& walkKickStep, Pose2f& step, const MotionPhase& lastPhase)) getLastWalkPhase,
});
