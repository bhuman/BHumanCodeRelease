/**
 * @file GoalPostPercept.h
 * Declaration of a struct that represents a goal post.
 * @author Lam Duy
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct GoalPostPercept
 * A struct that represents a detected goal post.
 */
STREAMABLE(GoalPostPercept,
{
  void draw() const,

  (Vector2i)(Vector2i::Zero()) positionInImage, /**< Position in the image. */
  (Vector2f)(Vector2f::Zero()) positionOnField, /**< Position relative to robot on the field. */
  (bool)(false) wasSeen, /**< Was seen? */
});
