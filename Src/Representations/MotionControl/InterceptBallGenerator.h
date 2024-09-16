/**
 * @file InterceptBallGenerator.h
 *
 * This file declares a representation that provides a function for motion to intercept the ball, by
 * - kicking a rolling ball, or
 * - walking into the rolling direction of the ball
 *
 * @author Florian Scholz
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(InterceptBallGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION;
  FUNCTION(Pose2f(const MotionRequest& motionRequest, const std::vector<Vector2f>& translationPolygon, const bool isLeftPhase, const std::optional<Vector2f>& oldStep)) intercept,
});
