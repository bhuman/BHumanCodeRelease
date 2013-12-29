/**
* @file GoalPercept.h
*
* Representation of a seen goal
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class GoalPost
* Description of a perceived goal post
*/
STREAMABLE(GoalPost,
{
public:
  ENUM(Position, IS_UNKNOWN, IS_LEFT, IS_RIGHT),

  (Position)(IS_UNKNOWN) position, /**< position of this post */
  (Vector2<int>) positionInImage, /**< The position of the goal post in the current image */
  (Vector2<float>) positionOnField, /**< The position of the goal post relative to the robot*/
});

/**
* @class GoalPercept
* Set of perceived goal posts
*/
STREAMABLE(GoalPercept,
{
public:
  /** Draws the perceived goal posts*/
  void draw() const,

  (std::vector<GoalPost>) goalPosts,
  (unsigned)(0) timeWhenGoalPostLastSeen, /**< Time when a goal post was seen. */
  (unsigned)(0) timeWhenCompleteGoalLastSeen, /**< Time when complete goal was seen. */
});
