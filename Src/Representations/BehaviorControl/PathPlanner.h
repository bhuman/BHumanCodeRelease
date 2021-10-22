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
   * The function plans a path to a target and returns the resulting obstacle avoidance information.
   * The parameters of the function are:
   * target The target to reach in field coordinates.
   * speed The speed to walk with in ratios of the maximum speeds.
   * The function returns the obstacle avoidance information.
   */
  FUNCTION(MotionRequest::ObstacleAvoidance(const Pose2f& target, const Pose2f& speed)) plan,
});
