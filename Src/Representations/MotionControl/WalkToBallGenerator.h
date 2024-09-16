/**
 * @file WalkToBallGenerator.h
 *
 * This file declares a representation that can create phases to walk to the ball.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Streaming/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include "Streaming/AutoStreamable.h"
#include <memory>

STREAMABLE(WalkToBallGenerator,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const Pose2f& targetInSCS, const Vector2f& ballInSCS, const int timeSinceBallWasSeen,
                                        const Pose2f& scsCognition, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                        const Pose2f& walkSpeed, const MotionPhase& lastPhase)) createPhase,
});
