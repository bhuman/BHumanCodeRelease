/**
 * @file OneVsOneDemoGameStateProvider.cpp
 *
 * This file implements a module that provides game control information without
 * an actual GameController. It is intended for one vs. one demonstration games.
 *
 * @author Daniel Krause
 * @author Arne Hasselbring
 */

#include "OneVsOneDemoGameStateProvider.h"
#include "Debugging/Debugging.h"
#include "Framework/Settings.h"
#include "Platform/SystemCall.h"
#include "Streaming/Global.h"

MAKE_MODULE(OneVsOneDemoGameStateProvider);

void OneVsOneDemoGameStateProvider::update(GameState& gameState)
{
  // v------ copied from GameStateProvider
  auto setToUnstiff = [&gameState]
  {
    gameState = GameState();
  };

  auto setToCalibration = [this, &gameState]
  {
    gameState = GameState();
    gameState.state = GameState::playing;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.playerState = GameState::calibration;
    gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
  };

  auto setToActive = [this, &gameState]
  {
    gameState = GameState();
    gameState.playerState = GameState::active;
    gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
  };

  // Declare here, because its use is not always executed.
  DECLARE_DEBUG_RESPONSE("module:OneVsOneDemoGameStateProvider:switchGamePhase");

  // This uses "module:GameStateProvider" on purpose so that startup scripts don't need to know which module runs.
  DEBUG_RESPONSE_ONCE("module:GameStateProvider:unstiff")
    setToUnstiff();

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:calibration")
    setToCalibration();

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:active")
    setToActive();

  bool ignoreChestButton = false;
  const bool unstiffButtonsPressed = theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
                                     (theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration ||
                                      theEnhancedKeyStates.pressedDuration[KeyStates::chest] > unstiffHeadButtonPressDuration) &&
                                     theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration;
  switch(gameState.playerState)
  {
    case GameState::unstiff:
      if(theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
      {
        setToActive();
        ignoreChestButton = true;
      }
      break;
    case GameState::calibration:
      if(unstiffButtonsPressed || theBehaviorStatus.calibrationFinished)
        setToUnstiff();
      break;
    default:
      if(unstiffButtonsPressed)
        setToUnstiff();
      else if(gameState.isInitial() &&
              gameState.playerState == GameState::active &&
              theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > calibrationHeadButtonPressDuration &&
              theEnhancedKeyStates.isPressedFor(KeyStates::chest, 1000u))
        setToCalibration();
  }
  // ^-------- end of copy

  // handle game state change by press and release two of the head touch sensor
  bool simRobotStateChange = false;
  DEBUG_RESPONSE_ONCE("module:OneVsOneDemoGameStateProvider:nextGameState")
    simRobotStateChange = true;

  // If this player is unstiff or in calibration mode, the game state stays reset.
  if(gameState.playerState == GameState::unstiff || gameState.playerState == GameState::calibration)
  {
    previousRightFootButtonPressed = false;
    previousComboButtonPressed = false;
    return;
  }

  if(simRobotStateChange || (previousComboButtonPressed && theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) >= stateChangeDelay &&
                             !theKeyStates.pressed[KeyStates::headRear] && !theKeyStates.pressed[KeyStates::headFront] && !theKeyStates.pressed[KeyStates::headMiddle] && !theKeyStates.pressed[KeyStates::chest]))
  {
    const bool isGoalkeeper = gameState.isGoalkeeper();
    gameState.timeWhenStateStarted = theFrameInfo.time;
    if(gameState.isPenaltyShootout())
    {
      if(gameState.isSet())
      {
        gameState.state = isGoalkeeper ? GameState::opponentPenaltyShot : GameState::ownPenaltyShot;
        gameState.timeWhenStateEnds = theFrameInfo.time + 30000;
      }
      else
      {
        gameState.state = isGoalkeeper ? GameState::waitForOpponentPenaltyShot : GameState::waitForOwnPenaltyShot;
        gameState.timeWhenStateEnds = 0;
      }
    }
    else if(gameState.isReady())
    {
      gameState.state = isGoalkeeper ? GameState::waitForOpponentKickOff : GameState::waitForOwnKickOff;
      gameState.timeWhenStateEnds = 0;
    }
    else if(gameState.isSet())
    {
      gameState.state = GameState::playing;
      gameState.timeWhenStateEnds = 0;
    }
    else
    {
      gameState.state = isGoalkeeper ? GameState::setupOpponentKickOff : GameState::setupOwnKickOff;
      gameState.timeWhenStateEnds = theFrameInfo.time + 45000;
    }
  }
  int checkKeyStates = theKeyStates.pressed[KeyStates::headMiddle] + theKeyStates.pressed[KeyStates::headRear] + theKeyStates.pressed[KeyStates::headFront];
  previousComboButtonPressed = checkKeyStates >= 2 || (previousComboButtonPressed && checkKeyStates >= 1);

  // change game state from ready to set when the robot is standing after some time has passed in ready
  if(gameState.isReady() && !gameState.isPenalized() && theMotionInfo.executedPhase == MotionPhase::stand &&
     theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > standingInReadyDelay)
  {
    gameState.state = gameState.isGoalkeeper() ? GameState::waitForOpponentKickOff : GameState::waitForOwnKickOff;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.timeWhenStateEnds = 0;
  }

  // change game state to STATE_PLAYING when the whistle has been detected
  if(gameState.isSet() && theWhistle.lastTimeWhistleDetected > gameState.timeWhenStateStarted)
  {
    gameState.state = gameState.isPenaltyShootout() ? (gameState.isGoalkeeper() ? GameState::opponentPenaltyShot : GameState::ownPenaltyShot) : GameState::playing;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.timeWhenStateEnds = gameState.isPenaltyShootout() ? theFrameInfo.time + 30000 : 0;
  }

  // Switch state from STATE_PLAYING to STATE_READY when the whistle has been detected.
  if(!gameState.isPenaltyShootout() && gameState.isPlaying() && theWhistle.lastTimeWhistleDetected > gameState.timeWhenStateStarted + whistleDelay)
  {
    gameState.state = gameState.isGoalkeeper() ? GameState::setupOpponentKickOff : GameState::setupOwnKickOff;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.timeWhenStateEnds = theFrameInfo.time + 45000;
    ++(gameState.isGoalkeeper() ? gameState.opponentTeam : gameState.ownTeam).score;
  }

  // switches between normal game and penalty shootout by pressing the right foot bumper
  if(gameState.isInitial())
  {
    bool rightFootButtonPressed = theKeyStates.pressed[KeyStates::rFootLeft] || theKeyStates.pressed[KeyStates::rFootRight];

    DEBUG_RESPONSE_ONCE("module:OneVsOneDemoGameStateProvider:switchGamePhase")
      rightFootButtonPressed = true;

    if(rightFootButtonPressed != previousRightFootButtonPressed && theFrameInfo.getTimeSince(whenRightFootButtonStateChanged) >= bumperDelay)
    {
      if(rightFootButtonPressed)
      {
        gameState.state = gameState.isPenaltyShootout() ? GameState::beforeHalf : GameState::beforePenaltyShootout;
        if(gameState.isPenaltyShootout())
          SystemCall::say(gameState.isGoalkeeper() ? "Penalty keeper" : "Penalty striker");
        else
          SystemCall::say("Normal");
      }
      previousRightFootButtonPressed = rightFootButtonPressed;
      whenRightFootButtonStateChanged = theFrameInfo.time;
    }
  }

  if(!ignoreChestButton && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
  {
    if(gameState.playerState == GameState::active)
      gameState.playerState = GameState::penalizedManual;
    else
      gameState.playerState = GameState::active;
    gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
  }

  gameState.phase = gameState.isPenaltyShootout() ? GameState::penaltyShootout : GameState::firstHalf;
  gameState.timeWhenPhaseEnds = theFrameInfo.time; // TODO
}
