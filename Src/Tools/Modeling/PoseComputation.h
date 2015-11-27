/**
 * @file PoseComputation.h
 *
 * Some functions that compute a robot pose on the field given
 * some observations of field elements (goals, center circle, ...)
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Math/Pose2f.h"

/**
 * @namespace PoseComputation
 *
 * Collection of functions
 */
namespace PoseComputation
{
  /**
   * Computes a pose on the field given the observation of two goalposts
   * @param pl The left goalpost (as seen by the robot)
   * @param pr The right goalpost (as seen by the robot)
   * @param plWorld The position of the left goalpost in the global frame of reference ("field coordinates")
   * @param prWorld The position of the right goalpost in the global frame of reference ("field coordinates")
   * @param goalpostRadius As you might have guessed ...
   * @return A pose
   */
  Pose2f computePoseFromTwoGoalposts(const Vector2f& pl, const Vector2f& pr, const Vector2f& plWorld,
                                     const Vector2f& prWorld, float goalpostRadius);
};