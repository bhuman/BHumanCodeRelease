/**
 * @file GoalPostsPercept.h
 *
 * Very simple representation of goal posts
 *
 * @autor Alpay Yildiray
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include <vector>

STREAMABLE(GoalPostsPercept,
{
  /**
   * Struct that represents a goal post.
   */
  STREAMABLE(GoalPost,
  {,
    (bool)(0) baseInImage,                  /**< Indicates if the base of the goal post lies inside of the image */
    (Vector2f) positionInImage,         /**< The position of the goal post in the current image */
    (int)(0) thicknessInImage,             /**< The thickness of the goal post in the current image */
    (int)(0) heightInImage,                /**< The height of the goal post in the current image */
    (Vector2f)(0, 0) relativePosition,  /**< The relative position of the goal post to the robot */
  });

  /**
   * Enum that specifies if one or both goal posts are being seen.
   */
  ENUM(Status,
  {,
    notSeen, /**< A goal post is not being seen. */
    oneSeen, /**< A goal post is being seen. */
    bothSeen, /** Both goal posts are being seen */
    guessed, /**< Not sure. */
  });

  GoalPostsPercept() = default;

  /** Draws the pillar*/
  void draw() const;

  /** Verifies that the pillar percept contains valid values. */
  void verify() const,

  (std::vector<GoalPost>) goalPosts, /** Contains a list of all detected goal posts (max 2) */
});

STREAMABLE_WITH_BASE(OtherGoalPostsPercept, GoalPostsPercept,
{,
});
