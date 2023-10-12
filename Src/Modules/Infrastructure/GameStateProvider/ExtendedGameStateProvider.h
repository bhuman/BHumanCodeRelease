/**
 * @file ExtendedGameStateProvider.h
 *
 * This file declares a module that provided information that is derived from the game state
 * but not suitable for being exchanged between threads.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Framework/Module.h"

MODULE(ExtendedGameStateProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  PROVIDES(ExtendedGameState),
  DEFINES_PARAMETERS(
  {,
    (int)(23000) minPenaltyTime, /**< The minimum time a robot must be penalized to actually believe it [ms]. */
    (int)(8000) minPenaltyTimeIPS, /**< The minimum time a robot must be penalized for Illegal Position in Set to actually believe it [ms]. */
  }),
});

class ExtendedGameStateProvider : public ExtendedGameStateProviderBase
{
  /**
   * This method updates the extended game state.
   * @param extendedGameState The updated representation.
   */
  void update(ExtendedGameState& extendedGameState) override;

  GameState::State stateLastFrame = GameState::beforeHalf; /**< The state of the game in the last frame. */
  GameState::PlayerState playerStateLastFrame = GameState::unstiff; /**< The state of this player in the last frame. */
  unsigned messageBudgetLastFrame = 1200u; /**< Message budget of the own team in the last frame. */
  unsigned timeWhenPenalized = 0; /**< Time when the current penalty started. */
};
