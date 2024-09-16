/**
 * @file WalkToPoseGenerator.h
 *
 * This file declares a representation that can create phases to walk to a pose.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(WalkToPoseGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION;

  FUNCTION(std::unique_ptr<MotionPhase>(const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                        const Pose2f& walkSpeed, bool isLeftPhase, const MotionPhase& lastPhase, const Pose2f& scsCognition,
                                        const bool isFastWalkAllowed, const std::optional<Vector2f>& targetOfInterest, const bool sideWalkAllowed)) createPhaseToTarget,
});
