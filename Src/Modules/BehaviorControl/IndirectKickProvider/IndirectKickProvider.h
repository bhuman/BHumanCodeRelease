/**
 * @file IndirectKickProvider.h
 *
 * TODO
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallKickInformation.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Math/RingBuffer.h"

MODULE(IndirectKickProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(MotionInfo),
  REQUIRES(TeamData),
  PROVIDES(IndirectKick),
  LOADS_PARAMETERS(
  {,
    (int) strikerTimeout,
  }),
});

class IndirectKickProvider : public IndirectKickProviderBase
{
private:
  GameState::State previousGameState = GameState::playing; /**< game state of the last frame */

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
};
