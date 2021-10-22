/**
 * @file WalkToPoseGenerator.h
 *
 * This file declares a representation that can create phases to walk to a pose.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(WalkToPoseGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION;

  FUNCTION(std::unique_ptr<MotionPhase>(const Pose2f&, const MotionRequest::ObstacleAvoidance&, const Pose2f&, bool, const MotionPhase& lastPhase, const bool isFastWalkAllowed)) createPhaseToTarget,
});
