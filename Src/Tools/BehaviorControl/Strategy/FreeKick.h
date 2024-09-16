/**
 * @file FreeKick.h
 *
 * This file declares the representation of a free kick.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "SetPlay.h"
#include "Math/Angle.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <limits>
#include <vector>

STREAMABLE_WITH_BASE(FreeKick, SetPlay,
{
  ENUM(BallSide,
  {,
    irrelevant, /**< The ball position does not matter. */
    left, /**< The ball is assumed to be in the left half of the field. */
    right, /**< The ball is assumed to be in the right half of the field. */
  });

  STREAMABLE(Condition,
  {,
    (float)(0.f) ballToOpponentGoalDistanceGE, /**< Lower bound on the distance between ball and (center of) the opponent's goal. */
    (float)(std::numeric_limits<float>::max()) ballToOpponentGoalDistanceLE, /**< Upper bound on the distance between ball and (center of) the opponent's goal. */
    (Angle)(0.f) ballToOpponentGoalAbsAngleGE, /**< Lower bound on the angle between x-axis and line between ball and (center of) the opponent's goal. */
    (Angle)(2 * pi) ballToOpponentGoalAbsAngleLE, /**< Upper bound on the angle between x-axis and line between ball and (center of) the opponent's goal. */
    (float)(std::numeric_limits<float>::lowest()) ballXGE, /**< Lower bound on the x-coordinate of the ball. */
    (float)(std::numeric_limits<float>::max()) ballXLE, /**< Upper bound on the x-coordinate of the ball. */
    (float)(std::numeric_limits<float>::lowest()) ballYAbsGE, /**< Lower bound on the absolute y-coordinate of the ball. */
    (float)(std::numeric_limits<float>::max()) ballYAbsLE, /**< Upper bound on the absolute y-coordinate of the ball. */
  }),

  (Condition) preconditions, /**< The conditions under which the free kick may be selected. */
  (Condition) invariants, /**< The conditions under which the free kick can stay selected. */
  (BallSide)(irrelevant) ballSide, /**< The side of the field in which the ball is assumed. If the ball is on the other side, everything will be mirrored. */
});

STREAMABLE_WITH_BASE(OwnFreeKick, FreeKick,
{
  ENUM(Type,
  {,
    ownCornerKick,
    ownCornerKickAttacking,
    ownGoalKick,
    ownKickInOwnHalf,
    ownKickInOpponentHalf,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(ownFreeKickBegin + static_cast<unsigned>(type));
  },
});

STREAMABLE_WITH_BASE(OpponentFreeKick, FreeKick,
{
  ENUM(Type,
  {,
    opponentCornerKick,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(opponentFreeKickBegin + static_cast<unsigned>(type));
  },
});
