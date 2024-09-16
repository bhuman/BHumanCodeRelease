/**
 * @file FieldInterceptBall.h
 *
 * Declaration of a representation that contains additional information
 * about the intercepted position of the ball that is required by the behavior.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(FieldInterceptBall,
{
  /** Debug drawings */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) interceptedEndPositionOnField, /**< The ball end position after interception in global field coordinates */
  (Vector2f)(Vector2f::Zero()) interceptedEndPositionRelative, /**< The ball end position after interception in relative coordinates */
  (Vector2f)(Vector2f::Zero()) intersectionPositionWithOwnYAxis,          /**< The position (in local coordinates) at which a rolling ball will pass the robot. Vector2f::Zero(), if this will not happen. */
  (Vector2f)(Vector2f::Zero()) intersectionPositionWithOwnXAxis,          /**< The position (in local coordinates) at which a rolling ball will pass the robot. Vector2f::Zero(), if this will not happen. */
  (float)(std::numeric_limits<float>::max()) timeUntilIntersectsOwnYAxis, /**< The time until a rolling ball will pass the robot. float::max, if this will not happen. */
  (float)(std::numeric_limits<float>::max()) timeUntilIntersectsOwnXAxis, /**< The time until a rolling ball will pass the robot. float::max, if this will not happen. */
  (bool)(false) interceptBall,                                            /**< If the rolling ball can be intercepted */
  (bool)(false) predictedInterceptBall,                                   /**< The ball will not intercept with us, but it is predicted it will happen. */
  (unsigned int)(0) lastInterceptBall,
});
