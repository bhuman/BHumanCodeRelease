/**
 * @file PathPlanner.h
 *
 * This file defines a representation that allows to determine a motion request
 * that brings the robot closer to a given target based on path planning.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Function.h"

STREAMABLE(PathPlanner,
{
  /**
   * The function plans a path to a target and returns the first step of it.
   * The parameters of the function are:
   * target The target to reach in field coordinates.
   * speed The speed to walk with in ratios of the maximum speeds.
   * excludePenaltyArea Avoid the own penalty area?
   * The function returns the motion request that should be executed.
   */
  FUNCTION(MotionRequest(const Pose2f& target, const Pose2f& speed, bool excludePenaltyArea)) plan,
});
