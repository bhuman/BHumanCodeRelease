/**
 * @file IndirectKickProvider.h
 *
 * This file declares a module that provides a representation that determines whether a Nao is allowed to shoot a goal.
 *
 * @author Ingo Kelm
 */

#pragma once

#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/SharedAutonomyRequest.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Framework/Module.h"

MODULE(IndirectKickProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(MotionInfo),
  REQUIRES(ReceivedTeamMessages),
  REQUIRES(SharedAutonomyRequest),
  PROVIDES(IndirectKick),
  LOADS_PARAMETERS(
  {,
    (bool) disableDirectKicks,
  }),
});

class IndirectKickProvider : public IndirectKickProviderBase
{
private:
  GameState::State previousGameState = GameState::playing;

  /**
   * Updates whether the next kick would be a direct kick or an indirect kick.
   * @param indirectKick The IndirectKick representation.
   */
  void update(IndirectKick& indirectKick) override;

  /**
   * Resets the indirect kick state.
   * @param indirectKick The IndirectKick representation.
   */
  void reset(IndirectKick& indirectKick);

  /**
   * Updates the Timestamps, when a player deliberately played a ball since the last reset (playing phase interruption). Index = player number - Settings::lowestValidPlayerNumber (1).
   * @param indirectKick The IndirectKick representation.
   */
  void updateLastBallContactTimestamps(IndirectKick& indirectKick);
};
