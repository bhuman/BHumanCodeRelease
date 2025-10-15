/**
 * @file FastestWalkStateProvider.h
 *
 * Provides the state for the fastest walk leaderboard challenge.
 *
 * @author Moritz Oppermann
 */

#include "Framework/Module.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/Leaderboards/FastestWalkState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(FastestWalkStateProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KeyStates),
  PROVIDES(FastestWalkState),
});

class FastestWalkStateProvider : public FastestWalkStateProviderBase
{
  void update(FastestWalkState& theFastestWalkState) override;

  FastestWalkState::WalkState state = FastestWalkState::WalkState::slalom;
  unsigned lastSwitch = 0;
  bool lastBumperState = false;
  unsigned walkStateIndex = 0;
};
