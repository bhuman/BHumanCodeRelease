/**
 * @file OneVsOneDemoGameStateProvider.h
 *
 * This file declares a module that provides a game state without an actual
 * GameController. It is intended for one vs. one demonstration games.
 *
 * @author Daniel Krause
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Framework/Module.h"

MODULE(OneVsOneDemoGameStateProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  REQUIRES(MotionInfo),
  REQUIRES(Whistle),
  PROVIDES(GameState),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(1000) unstiffHeadButtonPressDuration, /**< How long the head buttons need to be pressed until the robot transitions to unstiff (in ms). */
    (unsigned)(200) calibrationHeadButtonPressDuration, /**< How long the front head button needs to be pressed until the robot transitions to calibration (in ms). */
    (int)(500) stateChangeDelay, /**< Advancing the state with the head buttons is blocked for this time after a state change (in ms). */
    (int)(30) bumperDelay, /**< Some parameter for checking the foot bumper (in ms). */
    (int)(2000) standingInReadyDelay, /**< The game state switches to from ready to set if the robot is standing and ready has started more than this time ago (in ms). */
    (int)(2000) whistleDelay, /**< The transition from playing to ready using a whistle can only happen once this time has elapsed in playing (in ms). */
  }),
});

class OneVsOneDemoGameStateProvider : public OneVsOneDemoGameStateProviderBase
{
private:
  void update(GameState& gameState) override;

  bool previousComboButtonPressed = false;
  bool previousRightFootButtonPressed = false;
  unsigned whenRightFootButtonStateChanged = 0; /**< When last state change of the right foot bumper occurred. */
};
