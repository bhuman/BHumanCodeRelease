/**
 * @file GoalPostsPercept.h
 *
 * Very simple representation of goal posts
 *
 * @author Alpay Yildiray
 */

#pragma once

#include "Streaming/Enum.h"
#include "Math/Eigen.h"
#include <vector>

STREAMABLE(GoalPostsPercept,
{
  /**
   * Struct that represents a goal post.
   */
  STREAMABLE(GoalPost,
  {,
    (bool)(false) baseInImage,  /**< Indicates if the base of the goal post lies inside of the image */
    (Vector2f)(Vector2f::Zero()) positionInImage, /**< The position of the goal post in the current image */
    (Vector2f)(Vector2f::Zero()) positionOnField, /**< The relative position of the goal post to the robot */
    (Matrix2f)((Matrix2f() << 1.f, 0.f, 0.f, 1.f).finished()) covarianceOnField, /**< The measurement covariance of positionOnField */
  });

  GoalPostsPercept() = default;

  /** Draws the pillar*/
  void draw() const;

  /** Verifies that the pillar percept contains valid values. */
  void verify() const,

  (std::vector<GoalPost>) goalPosts, /**< Contains a list of all detected goal posts  */
});
