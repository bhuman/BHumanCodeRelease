/**
 * @file BallDropInModel.h
 *
 * This file declares a representation that contains information about ball out / drop in.
 *
 * @author Arne Hasselbring, Nicole Schrader
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(BallDropInModel,
{
  ENUM(DropInType,
  {,
    ownGoalKick,
    ownCornerKick,
    opponentGoalKick,
    opponentCornerKick,
    kickIn,
  });

  /** Draws the representation. */
  void draw() const,

  (bool)(false) isValid, /**< Whether the current drop in position is valid (can be invalid because GameController says the ball went out but no robot touched it recently). */
  (unsigned)(0) dropInTypes, /**< The set of possible methods how the ball reenters the field. */
  (unsigned)(0) lastTimeWhenBallOutWasObserved, /**< The last time when the ball was observed to go out. */
  (unsigned)(0) lastTimeWhenBallWentOut, /**< The last time when the ball went out according to the GameController. */
  (unsigned)(0) lastTimeWhenBallWasOnTheField, /**< The last time when the ball was known to be on the field. */
  (Vector2f)(Vector2f::Zero()) ballDirection, /**< The last direction of the ball*/
  (Vector2f)(Vector2f::Zero()) outPosition, /**< The position where the ball was observed to go out. */
  (std::vector<Vector2f>)({}) dropInPositions, /**< The expected positions where the ball can be put in again. */
});
