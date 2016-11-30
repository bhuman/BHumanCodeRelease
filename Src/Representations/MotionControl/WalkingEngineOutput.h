/**
 * @file Representations/MotionControl/WalkingEngineOutput.h
 * This file declares a struct that represents the output of modules generating motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Representations/MotionControl/WalkRequest.h"

/**
 * @struct WalkingEnigeOutput
 * A struct that represents the output of the walking engine.
 */
STREAMABLE(WalkingEngineOutput,
{
  ENUM(SupportFoot,
  {,
    left,
    right,
    both,
  }),

  (bool)(true) standing, /**< Whether the robot is standing or walking */
  (Pose2f) speed, /**< The current walking speed in mm/s and rad/s. */
  (Pose2f)(Pose2f(45_deg, 100.f, 100.f)) maxSpeed, /**< The maximum walking speed in mm/s and rad/s. */
  (Pose2f) odometryOffset, /**< The body motion performed in this step. */
  (Pose2f) upcomingOdometryOffset, /**< The remaining odometry offset for the currently executed step. */
  (bool)(false) upcomingOdometryOffsetValid, /**< Whether the \c upcomingOdometryOffset is precise enough to be used */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (WalkRequest) executedWalk, /**< The walk currently executed. */
  (SupportFoot)(both) supportFoot,
});
