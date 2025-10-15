/**
 * @file BestKickStateProvider.h
 *
 * Provides the state for the best kick leaderboard challenge.
 *
 * @author Moritz Oppermann
 */

#include "Framework/Module.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/Leaderboards/BestKickState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(BestKickStateProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KeyStates),
  PROVIDES(BestKickState),
});

class BestKickStateProvider : public BestKickStateProviderBase
{
  void update(BestKickState& theBestKickState) override;

  BestKickState::Target target = BestKickState::Target::a;
  unsigned lastSwitch = 0;
  bool lastBumperState = false;
  unsigned targetIndex = 0;
};
