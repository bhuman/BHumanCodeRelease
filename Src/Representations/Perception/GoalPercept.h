/**
 * @file GoalPercept.h
 *
 * Representation of a seen goal
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct GoalPost
 * Description of a perceived goal post
 */
STREAMABLE(GoalPost,
{
  ENUM(Position,
  {,
    IS_UNKNOWN,
    IS_LEFT,
    IS_RIGHT,
  }),

  (Position)(IS_UNKNOWN) position, /**< position of this post */
  (Vector2i) positionInImage, /**< The position of the goal post in the current image */
  (Vector2f) positionOnField, /**< The position of the goal post relative to the robot*/
});

/**
 * @struct GoalPercept
 * Set of perceived goal posts
 */
STREAMABLE(GoalPercept,
{
  /** Draws the perceived goal posts*/
  void draw() const,

  (std::vector<GoalPost>) goalPosts, /**<Is empty if no goal posts where seen this frame */
  (unsigned)(0) timeWhenGoalPostLastSeen, /**< Time when a goal post was seen. */
  (unsigned)(0) timeWhenCompleteGoalLastSeen, /**< Time when complete goal was seen. */
});
