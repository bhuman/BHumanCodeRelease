/**
 * @file WalkKickGenerator.h
 *
 * This file declares a representation that generates walk kicks.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Motion/PreStepType.h"
#include "Tools/Motion/WalkKickVariant.h"
#include "Streaming/AutoStreamable.h"
#include <memory>

STREAMABLE(WalkKickGenerator,
{
  FUNCTION(bool(WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange, const PreStepType preStepType, const bool turnKickAllowed)) canStart; /**< Checks whether the specified kick can start in the upcoming phase. */
  FUNCTION(std::unique_ptr<MotionPhase>(const WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const bool playSound)) createPhase; /**< Creates a phase to execute the specified kick. */
  FUNCTION(Pose2f(const bool isLeftPhase, const Angle direction, const float stealXShift)) getVShapeWalkStep;
  FUNCTION(void(const Pose2f& step)) drawStep;
  FUNCTION(bool(WalkKickStep& walkKickStep, const float timeSinceStepStarted)) dynamicWalkKickStepUpdate,
});
