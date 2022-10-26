/**
 * @file BallPossession.h
 *
 * This file declares a represention which contains an estimate
 * which team has possession of the ball.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(BallPossession,
{
  ENUM(State,
  {,
    unknown, /**< Not known, e.g. because the position of the ball is unknown. */
    own, /**< Our team has possession of the ball. */
    opponent, /**< The opponent team has possession of the ball. */
    contended, /**< The ball is contended. */
    loose, /**< The ball is far from any player. */
  }),

  (State)(unknown) state, /**< The currently estimated state of ball possession. */
});
