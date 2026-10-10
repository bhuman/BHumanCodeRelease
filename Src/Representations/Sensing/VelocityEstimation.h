/**
 * @file VelocityEstimation.h
 * Declaration of struct VelocityEstimation. It contains an estimation about the current velocity of the robot.
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"

/**
 * Estimated velocity of the robot in degree/s and mm/s
 */
STREAMABLE_WITH_BASE(VelocityEstimation, Pose2f,
{
  VelocityEstimation() = default,

  (Pose2f) highestLastQuarterSecond,
});
