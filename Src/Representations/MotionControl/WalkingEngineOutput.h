/**
 * @file Representations/MotionControl/WalkingEngineOutput.h
 * This file declares a struct that represents the output of modules generating motion.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct WalkingEngineOutput
 * A struct that represents the output of the walking engine.
 */
STREAMABLE_WITH_BASE(WalkingEngineOutput, JointRequest,
{,
  (bool)(true) standing, /**< Whether the robot is standing or walking */
  (Pose2f) speed, /**< The current walking speed in mm/s and rad/s. */
  (Pose2f)(45_deg, 100.f, 100.f) maxSpeed, /**< The maximum walking speed in mm/s and rad/s. */
  (Pose2f) odometryOffset, /**< The body motion performed in this step. */
  (Pose2f) upcomingOdometryOffset, /**< The minimum remaining odometry offset until the robot can come to a full stop. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (WalkRequest) executedWalk, /**< The walk currently executed. */
});
