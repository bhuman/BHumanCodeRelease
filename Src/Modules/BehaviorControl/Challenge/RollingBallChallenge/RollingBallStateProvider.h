/**
 * @file RollingBallStateProvider.h
 *
 * Provides the state for the rolling ball challenge.
 *
 * @author Philip Reichenberg
 */

#include "Framework/Module.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/Challenge/RollingBallState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(RollingBallStateProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KeyStates),
  PROVIDES(RollingBallState),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) sideSwitchTimeout, /**< Switch ramp side only once during this time. */
    (int)(500) rampDistanceTimeout, /**< Ramp distance can only be changed once during this time. */
    (Rangef)(300.f, 3000.f) rampDistanceRange, /**< Ramp distance range. */
    (float)(300.f) rampDistanceStepsize, /**< Ramp distance steps. */
  }),
});

class RollingBallStateProvider : public RollingBallStateProviderBase
{
  void update(RollingBallState& theRollingBallState) override;

  unsigned headFrontTimestamp = 0;
  unsigned headRearTimestamp = 0;
};
