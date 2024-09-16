/**
 * @file Strategy.h
 *
 * This file declares the description of a strategy.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "ActiveRole.h"
#include "FreeKick.h"
#include "KickOff.h"
#include "PenaltyKick.h"
#include "Tactic.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <vector>

STREAMABLE(Strategy,
{
  ENUM(Type,
  {,
    none,
    attacking7v7,
    attackingSAC2v2,
    defendingSAC2v2,
  });

  template<typename SetPlayType>
  STREAMABLE(WeightedSetPlay,
  {,
    (typename SetPlayType::Type)(SetPlayType::numOfTypes) type, /**< The set play. */
    (float)(1.f) weight, /**< The weight of this set play, representing the selection probability or rank when being applicable. */
  });

  STREAMABLE(TacticState,
  {
    STREAMABLE(Transition,
    {
      STREAMABLE(Condition,
      {,
        (unsigned)(0) numOfFieldPlayersGE, /**< Lower bound on the number of field players. */
        (unsigned)(std::numeric_limits<unsigned>::max()) numOfFieldPlayersLE, /**< Upper bound on the number of field players. */
        (unsigned)(0) ownScoreGE, /**< Lower bound on the own score. */
        (unsigned)(std::numeric_limits<unsigned>::max()) ownScoreLE, /**< Upper bound on the own score. */
        (unsigned)(0) opponentScoreGE, /**< Lower bound on the opponent's score. */
        (unsigned)(std::numeric_limits<unsigned>::max()) opponentScoreLE, /**< Upper bound on the opponent's score. */
        (int)(std::numeric_limits<int>::min()) scoreDifferenceGE, /**< Lower bound on the score difference (own - opponent's). */
        (int)(std::numeric_limits<int>::max()) scoreDifferenceLE, /**< Upper bound on the score difference (own - opponent's). */
        (float)(0.f) ballXThreshold, /**< An x coordinate to evaluate the following conditions. */
        (int)(0) timeSinceBallBehindThresholdGE, /**< Lower bound on the time the ball has been behind the above threshold. */
        (int)(0) timeSinceBallAheadOfThresholdGE, /**< Lower bound on the time the ball has been ahead of the above threshold. */
        (std::optional<bool>) sacAlternateTactic, /**< Play alternate tactic in the shared autonomy challenge. */
      }),
      (Tactic::Type) to, /**< The tactic to switch to. */
      (std::vector<Condition>) conditions, /**< The conditions under which the switch may occur (only one of them must hold). */
    }),
    (Tactic::Type) tactic, /**< The tactic in this state. */
    (std::vector<Transition>) transitions, /**< The possible transitions from this tactic. */
  }),

  (std::vector<TacticState>) tactics, /**< The tactic state machine in this strategy. */
  (std::vector<WeightedSetPlay<OwnKickOff>>) ownKickOffs, /**< The available kick-offs. */
  (std::vector<WeightedSetPlay<OpponentKickOff>>) opponentKickOffs, /**< The available kick-off defenses. */
  (std::vector<WeightedSetPlay<OwnPenaltyKick>>) ownPenaltyKicks, /**< The available penalty kicks. */
  (std::vector<WeightedSetPlay<OpponentPenaltyKick>>) opponentPenaltyKicks, /**< The available penalty kick defenses. */
  (std::vector<WeightedSetPlay<OwnFreeKick>>) ownFreeKicks, /**< The available free kicks. */
  (std::vector<WeightedSetPlay<OpponentFreeKick>>) opponentFreeKicks, /**< The available free kick defenses. */
});
