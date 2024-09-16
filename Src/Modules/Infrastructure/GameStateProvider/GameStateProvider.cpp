/**
 * @file GameStateProvider.cpp
 *
 * This file implements a module that provides a condensed representation of the game state.
 *
 * @author Arne Hasselbring
 */

#include "GameStateProvider.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <algorithm>

MAKE_MODULE(GameStateProvider);

void GameStateProvider::update(GameState& gameState)
{
  DECLARE_DEBUG_DRAWING("module:GameStateProvider:ballCovariance", "drawingOnField");
  auto setToUnstiff = [this, &gameState]
  {
    reset(gameState);
  };

  auto setToCalibration = [this, &gameState]
  {
    reset(gameState);
    gameState.state = GameState::playing;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.playerState = GameState::calibration;
    gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
  };

  auto setToActive = [this, &gameState]
  {
    reset(gameState);
    gameState.playerState = GameState::active;
    gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
    timeOfLastIntegratedGameControllerData = theFrameInfo.time - gameControllerTimeout;
  };

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:unstiff")
    setToUnstiff();

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:calibration")
    setToCalibration();

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:active")
  {
    setToActive();
    timeOfLastIntegratedGameControllerData = theFrameInfo.time - 1;
  }

  if(gameState.state != GameState::afterHalf)
    timeWhenStateNotAfterHalf = theFrameInfo.time;

  gameState.whistled = false;
  bool ignoreChestButton = false;
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
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theBehaviorStatus.calibrationFinished)
        setToUnstiff();
      break;
    default:
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         (SystemCall::getMode() != SystemCall::simulatedRobot &&
          theFrameInfo.getTimeSince(timeWhenStateNotAfterHalf) > unstiffAfterHalfDuration))
        setToUnstiff();
      else if(gameState.isInitial() &&
              gameState.playerState == GameState::active &&
              theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > calibrationHeadButtonPressDuration &&
              theEnhancedKeyStates.isPressedFor(KeyStates::chest, 1000u))
        setToCalibration();
  }

  updateBallBuffer(gameState);
  updateIllegalMotionInSetTimestamps();
  updateIllegalPosition();

  // If this player is unstiff or in calibration mode, the game state stays reset and GameController messages are ignored.
  if(gameState.playerState == GameState::unstiff || gameState.playerState == GameState::calibration)
    return;

  // Update whether the GameController is "active".
  const bool useGameControllerData = theGameControllerData.timeLastPacketReceived > timeOfLastIntegratedGameControllerData;
  if(useGameControllerData)
    gameState.gameControllerActive = true;
  else if(theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) >= gameControllerTimeout)
    gameState.gameControllerActive = false;

  // Reset whistle state transitions that turn out to be wrong.
  if(gameStateOverridden)
  {
    switch(gameState.state)
    {
      case GameState::setupOwnKickOff:
      case GameState::setupOpponentKickOff:
        // A guessed READY state (after a goal) is reverted if
        // - sufficient time has elapsed without the GameController state actually changing to READY or
        // - the GameController continues to announce a set play.
        // TODO: Instead of checking getTimeSince(timeWhenStateStarted), we should check the time since
        //       the last whistle which would have made us switch to READY, because it could be that the
        //       *original* whistle which triggered the transition to READY was wrong, but a *later* whistle
        //       was true.
        if(gameState.gameControllerActive && theGameControllerData.state != STATE_READY && !gameState.kickOffSetupFromTouchlines &&
           (theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > goalSignalDelay + gameControllerOperatorDelay ||
            (theGameControllerData.setPlay != SET_PLAY_NONE && theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > gameControllerOperatorDelay)))
        {
          // Even if the game state had been overridden when the (wrong, as it turns out) transition to
          // READY was triggered, that should not be the case anymore because
          // - in the case of "sufficient time", that state would be sent truthfully now, too (as long as goalSignalDelay >= playingSignalDelay), and
          // - in case a set play is sent, it is true information.
          gameStateOverridden = false;
          minWhistleTimestamp = std::max(gameState.timeWhenStateStarted, theFrameInfo.time - acceptPastWhistleDelay);
          gameState.state = convertGameControllerDataToState(theGameControllerData);
          gameState.timeWhenStateStarted = timeWhenStateStartedBeforeWhistle; // TODO: We could be in a completely different state now than we were when we started guessing.
          gameState.timeWhenStateEnds = 0; // TODO: We could be in a state that requires this to be != 0.
        }
        break;
      case GameState::ownKickOff:
      case GameState::opponentKickOff:
      case GameState::ownPenaltyKick:
      case GameState::opponentPenaltyKick:
      case GameState::ownPenaltyShot:
      case GameState::opponentPenaltyShot:
      case GameState::playing:
        // A guessed PLAYING state is reverted if a player has been penalized for illegal motion in SET.
        // We do not include a timeout here because as soon as a robot moves, it is penalized anyway.
        if(gameState.gameControllerActive && theGameControllerData.state == STATE_SET &&
           checkForIllegalMotionInSetPenalty(timeWhenSwitchedToPlayingViaWhistle))
        {
          gameStateOverridden = false;
          minWhistleTimestamp = theFrameInfo.time;
          gameState.state = convertGameControllerDataToState(theGameControllerData);
          gameState.timeWhenStateStarted = timeWhenStateStartedBeforeWhistle;
          gameState.timeWhenStateEnds = 0;
        }

        // A guessed PLAYING state is reverted if a player has been penalized for illegalPosition or the ball returned to a legal position.
        auto gameControllerState = convertGameControllerDataToState(theGameControllerData);
        if(GameState::isFreeKick(gameControllerState) &&
           ((*std::max_element(illegalPositionTimestampsOwnTeam.begin(), illegalPositionTimestampsOwnTeam.end()) > gameState.timeWhenStateStarted &&
             GameState::isForOpponentTeam(gameControllerState)) ||
            (*std::max_element(illegalPositionTimestampsOpponentTeam.begin(), illegalPositionTimestampsOpponentTeam.end()) > gameState.timeWhenStateStarted &&
             GameState::isForOwnTeam(gameControllerState)) ||
            theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > gameControllerOperatorDelay ||
            (checkBallLegal(gameControllerState, confidenceIntervalToLegalPositionNotFree) &&
             !GameState::isPushingFreeKick(gameControllerState))))
        {
          gameStateOverridden = false;
          gameState.state = gameControllerState;
          gameState.timeWhenStateStarted = timeWhenStateStartedBeforeBallMoved;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + freeKickDuration;
        }

        // A guessed PLAYING state is reverted to penaltyKick if a player has been penalized for illegalPosition.
        // The gameControllerOperatorDelay is not considered, because the game controller might still send SET to hide the whistle
        if((GameState::isPenaltyKick(gameControllerState) && !GameState::isReady(gameControllerState)) &&
           (*std::max_element(illegalPositionTimestampsOwnTeam.begin(), illegalPositionTimestampsOwnTeam.end()) > gameState.timeWhenStateStarted ||
            *std::max_element(illegalPositionTimestampsOpponentTeam.begin(), illegalPositionTimestampsOpponentTeam.end()) > gameState.timeWhenStateStarted))
        {
          gameState.state = GameState::isForOwnTeam(gameControllerState) ? GameState::ownPenaltyKick : GameState::opponentPenaltyKick;
          gameState.timeWhenStateStarted = timeWhenStateStartedBeforeBallMoved;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickDuration;
        }
        break;
    }
  }

  // A kick-off or penalty-kick is over once either the ball has moved, is no longer on a legal position or the time has elapsed.
  if((gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff || gameState.state == GameState::ownPenaltyKick ||
      gameState.state == GameState::opponentPenaltyKick) &&
     (checkBallHasMoved(gameState) || theFrameInfo.time >= gameState.timeWhenStateEnds ||
      !checkBallLegal(gameState.state, confidenceIntervalToLegalPositionFree)))
  {
    gameStateOverridden = true;
    timeWhenStateStartedBeforeBallMoved = gameState.timeWhenStateStarted;
    gameState.state = GameState::playing;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.timeWhenStateEnds = 0;
  }

  // State transitions that are triggered by the whistle:
  if(!gameState.gameControllerActive || !theGameControllerData.isTrueData)
  {
    switch(gameState.state)
    {
      case GameState::opponentKickIn:
      case GameState::opponentCornerKick:
      case GameState::opponentPushingFreeKick:
      case GameState::opponentGoalKick:
      case GameState::ownPushingFreeKick:
      case GameState::ownKickIn:
      case GameState::ownGoalKick:
      case GameState::ownCornerKick:
        // Check for ball free
        if(checkBallHasMoved(gameState) || !checkBallLegal(gameState.state, confidenceIntervalToLegalPositionFree))
        {
          gameStateOverridden = true;
          timeWhenStateStartedBeforeBallMoved = gameState.timeWhenStateStarted;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          gameState.timeWhenStateEnds = 0;
          gameState.state = GameState::playing;
        }
        [[fallthrough]];
      case GameState::playing:
      case GameState::ownKickOff:
      case GameState::opponentKickOff:
      case GameState::ownPenaltyKick:
      case GameState::opponentPenaltyKick:
        // The referee signals the transition to READY after a goal with a whistle.
        // There is a "cooldown period" in which whistles are ignored to avoid confusion with the whistle that started PLAYING.
        // Since goals from penalty kicks can be scored quicker, the have a different (shorter) cooldown period.
        if(checkWhistleForGoal &&
            checkForWhistle(std::max(minWhistleTimestamp
                           + ((gameState.state == GameState::ownPenaltyKick || gameState.state == GameState::opponentPenaltyKick)
                              ? ignoreWhistleAfterPenaltyKick : ignoreWhistleAfterKickOff),
                                     theBallInGoal.lastTimeInGoal > acceptWhistleBeforeGoal ? theBallInGoal.lastTimeInGoal - acceptWhistleBeforeGoal : 0u),
                            gameState.gameControllerActive) &&
            (theFrameInfo.getTimeSince(theBallInGoal.lastTimeInGoal) <= acceptBallInGoalDelay || !checkBallForGoal))
        {
          // Remember start of previous state in case the whistle detection turns out to be wrong.
          gameStateOverridden = true;
          timeWhenStateStartedBeforeWhistle = gameState.timeWhenStateStarted;
          gameState.kickOffSetupFromTouchlines = false;
          gameState.state = theBallInGoal.inOwnGoal ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
          ++(theBallInGoal.inOwnGoal ? gameState.opponentTeam : gameState.ownTeam).score;
          gameState.whistled = true;
        }
        break;
      case GameState::waitForOwnKickOff:
      case GameState::waitForOpponentKickOff:
      case GameState::waitForOwnPenaltyKick:
      case GameState::waitForOpponentPenaltyKick:
      case GameState::waitForOwnPenaltyShot:
      case GameState::waitForOpponentPenaltyShot:
        if(checkForWhistle(minWhistleTimestamp, gameState.gameControllerActive))
        {
          // Remember start of previous state in case the whistle detection turns out to be wrong.
          gameStateOverridden = true;
          timeWhenStateStartedBeforeWhistle = gameState.timeWhenStateStarted;
          timeWhenSwitchedToPlayingViaWhistle = theFrameInfo.time;
          minWhistleTimestamp = theFrameInfo.time;
          switch(gameState.state)
          {
            case GameState::waitForOwnKickOff:
              gameState.state = GameState::ownKickOff;
              break;
            case GameState::waitForOpponentKickOff:
              gameState.state = GameState::opponentKickOff;
              break;
            case GameState::waitForOwnPenaltyKick:
              gameState.state = GameState::ownPenaltyKick;
              break;
            case GameState::waitForOpponentPenaltyKick:
              gameState.state = GameState::opponentPenaltyKick;
              break;
            case GameState::waitForOwnPenaltyShot:
              gameState.state = GameState::ownPenaltyShot;
              break;
            case GameState::waitForOpponentPenaltyShot:
              gameState.state = GameState::opponentPenaltyShot;
              break;
          }
          gameState.timeWhenStateStarted = theFrameInfo.time;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + (gameState.isKickOff() ? kickOffDuration : penaltyKickDuration);
          gameState.whistled = true;
        }
        break;
    }
  }

  // Transition from Initial to Ready
  if(gameState.state == GameState::standby && theInitialToReady.isTransition())
  {
    const bool isKickingTeam = theGameControllerData.kickingTeam == Global::getSettings().teamNumber;
    gameState.state = isKickingTeam ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
    gameState.kickOffSetupFromTouchlines = true;
    gameStateOverridden = true;
  }

  if(useGameControllerData)
  {
    gameState.competitionPhase = static_cast<GameState::CompetitionPhase>(theGameControllerData.competitionPhase);

    // The index of the own team within theGameControllerData.
    const std::size_t ownTeamIndex = theGameControllerData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1;
    gameState.leftHandTeam = ownTeamIndex == 0;

    // Everything else would be a bug in the GameControllerDataProvider:
    ASSERT(theGameControllerData.teams[ownTeamIndex].teamNumber == Global::getSettings().teamNumber);

    // The game phase is always reported truthfully by the GameController.
    // See below for timeWhenPhaseEnds.
    gameState.phase = convertGameControllerDataToPhase(theGameControllerData);

    auto gameControllerState = convertGameControllerDataToState(theGameControllerData);
    // When the guessed state and the state from the GameController match, we can trust the GameController again.
    gameStateOverridden &= gameState.state != gameControllerState;
    // States other than SET, PLAYING or STANDBY are always true.
    gameStateOverridden &= GameState::isSet(gameControllerState) || GameState::isPlaying(gameControllerState) || gameControllerState == GameState::standby;
    // SET is true when switching there from READY.
    gameStateOverridden &= !GameState::isSet(gameControllerState) || !gameState.isReady();
    // Free Kicks are mostly true, but not if we guess READY (because it might be that the GameController still sends a free kick when a goal is scored).
    //gameStateOverridden &= !GameState::isFreeKick(gameControllerState) || gameState.isReady();
    if(!gameStateOverridden && gameControllerState != gameState.state)
    {
      // Ignore older whistles when switching to some states.
      if((!gameState.isSet() && GameState::isSet(gameControllerState)) ||
         (!gameState.isPlaying() && GameState::isPlaying(gameControllerState)))
        minWhistleTimestamp = theFrameInfo.time;

      // kickOffSetupFromTouchlines: We are switching from some INITIAL state to READY and we already integrated a GameController packet recently.
      if((gameControllerState == GameState::setupOwnKickOff || gameControllerState == GameState::setupOpponentKickOff) &&
         gameState.isInitial() && theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) < gameControllerTimeout)
        gameState.kickOffSetupFromTouchlines = true;

      // Actually set the new state and associated timestamps:
      gameState.state = gameControllerState;
      gameState.timeWhenStateStarted = theFrameInfo.time;
      switch(gameControllerState)
      {
        case GameState::setupOwnKickOff:
        case GameState::setupOpponentKickOff:
          // Switching to READY while changing the score means we haven't heard the whistle after a goal
          // or switching to READY when the KickOff-Setup starts from the touchline means we haven't detected the referee gesture.
          // In those cases, the state actually began earlier.
          if(!theGameControllerData.isTrueData &&
             ((theGameControllerData.teams[ownTeamIndex].score != gameState.ownTeam.score ||
               theGameControllerData.teams[1 - ownTeamIndex].score != gameState.opponentTeam.score) ||
              gameState.kickOffSetupFromTouchlines))
            gameState.timeWhenStateStarted -= (kickOffSetupDuration - theGameControllerData.secondaryTime * 1000);
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
          break;
        case GameState::ownKickOff:
        case GameState::opponentKickOff:
          if(!theGameControllerData.isTrueData)
            gameState.timeWhenStateStarted -= (kickOffDuration - theGameControllerData.secondaryTime * 1000);
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffDuration;
          break;
        case GameState::setupOwnPenaltyKick:
        case GameState::setupOpponentPenaltyKick:
          // The GameController never delays this state change.
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickSetupDuration;
          break;
        case GameState::ownPenaltyKick:
        case GameState::opponentPenaltyKick:
        case GameState::ownPenaltyShot:
        case GameState::opponentPenaltyShot:
          // Penalty kicks (whether in a game or a penalty shoot-out) are delayed by the GameController.
          if(!theGameControllerData.isTrueData)
            gameState.timeWhenStateStarted -= playingSignalDelay;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickDuration;
          break;
        case GameState::ownPushingFreeKick:
        case GameState::opponentPushingFreeKick:
        case GameState::ownKickIn:
        case GameState::opponentKickIn:
        case GameState::ownGoalKick:
        case GameState::opponentGoalKick:
        case GameState::ownCornerKick:
        case GameState::opponentCornerKick:
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + freeKickDuration;
          break;
        default:
          // All other states don't have a fixed duration.
          gameState.timeWhenStateEnds = 0;
          break;
      }
    }

    const auto fillTeam = [&](const RoboCup::TeamInfo& teamInfo, bool speculatingGoalFor, GameState::Team& team)
    {
      static_assert(std::tuple_size<decltype(team.playerStates)>::value == MAX_NUM_PLAYERS);
      for(std::size_t i = 0; i < team.playerStates.size(); ++i)
        team.playerStates[i] = convertPenaltyToPlayerState(teamInfo.players[i].penalty);
      team.number = teamInfo.teamNumber;
      team.fieldPlayerColor = static_cast<GameState::Team::Color>(teamInfo.fieldPlayerColor);
      team.goalkeeperColor = static_cast<GameState::Team::Color>(teamInfo.goalkeeperColor);
      team.goalkeeperNumber = teamInfo.goalkeeper;
      team.score = teamInfo.score + (speculatingGoalFor ? 1 : 0);
      team.messageBudget = teamInfo.messageBudget;
    };

    fillTeam(theGameControllerData.teams[ownTeamIndex], gameState.state == GameState::setupOpponentKickOff && gameControllerState != gameState.state, gameState.ownTeam);
    fillTeam(theGameControllerData.teams[1 - ownTeamIndex], gameState.state == GameState::setupOwnKickOff && gameControllerState != gameState.state, gameState.opponentTeam);

    const auto gameControllerPlayerState = convertPenaltyToPlayerState(theGameControllerData.teams[ownTeamIndex].players[Global::getSettings().playerNumber - 1].penalty);
    if(gameControllerPlayerState != gameState.playerState)
    {
      gameState.playerState = gameControllerPlayerState;
      // If this is the first GameController packet for some time, it might be that the robot actually has had the state for a longer time.
      if(theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) >= gameControllerTimeout)
        gameState.timeWhenPlayerStateStarted = 0;
      else
        gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
    }

    gameState.timeWhenPenaltyEnds = gameState.isPenalized()
                                    ? theGameControllerData.timeLastPacketReceived
                                      + theGameControllerData.teams[ownTeamIndex].players[Global::getSettings().playerNumber - 1].secsTillUnpenalised * 1000
                                    : 0;

    timeOfLastIntegratedGameControllerData = theGameControllerData.timeLastPacketReceived;
  }
  else if(!gameState.gameControllerActive)
  {
    // Button interface (only when we haven't received from a GameController recently):
    // - chest button to switch between manual penalty and PLAYING / penalty shoot-out
    // - right foot bumper to switch between penalty shoot-out and normal mode
    if(!ignoreChestButton && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
    {
      if(gameState.playerState == GameState::active)
        gameState.playerState = GameState::penalizedManual;
      else
      {
        if(gameState.phase == GameState::penaltyShootout)
        {
          // When a penalty shoot-out is done using the button interface (which cannot
          // happen in regular games, since the GameController *must* be used there),
          // the state never switches to afterPenaltyShot/waitForPenaltyShot. However,
          // the SelfLocator handles this as it also resets the pose when the robot is
          // unpenalized (the SideInformationProvider doesn't, but its output isn't
          // used during penalty shoot-outs).
          gameState.state = manualPenaltyShootoutForOwnTeam ? GameState::ownPenaltyShot : GameState::opponentPenaltyShot;
          gameState.timeWhenStateStarted = theFrameInfo.time - playingSignalDelay;
          gameState.timeWhenPhaseEnds = gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickDuration;
        }
        else
        {
          minWhistleTimestamp = theFrameInfo.time;
          gameState.state = GameState::playing;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          gameState.timeWhenStateEnds = 0;
        }
        gameState.playerState = GameState::active;
      }
      gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
    }

    if(gameState.isInitial() && gameState.playerState == GameState::active)
    {
      if(theEnhancedKeyStates.hitStreak[KeyStates::rFootRight] == 1)
      {
        if(gameState.phase != GameState::penaltyShootout)
        {
          gameState.phase = GameState::penaltyShootout;
          gameState.timeWhenPhaseEnds = 0;
          gameState.state = GameState::beforePenaltyShootout;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          gameState.timeWhenStateEnds = 0;
          manualPenaltyShootoutForOwnTeam = true;
          SystemCall::say("Penalty striker");
        }
        else if(manualPenaltyShootoutForOwnTeam)
        {
          manualPenaltyShootoutForOwnTeam = false;
          SystemCall::say("Penalty keeper");
        }
        else
        {
          gameState.phase = GameState::firstHalf;
          gameState.timeWhenPhaseEnds = 0;
          gameState.state = GameState::beforeHalf;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          gameState.timeWhenStateEnds = 0;
          SystemCall::say("Soccer");
        }
      }
    }
  }

  if(gameState.gameControllerActive && !gameStateOverridden)
  {
    // This is done this way so that old code that was using secsRemaining still works:
    gameState.timeWhenPhaseEnds = theFrameInfo.time + theGameControllerData.secsRemaining * 1000;
  }

  if(gameState.state != GameState::setupOwnKickOff && gameState.state != GameState::setupOpponentKickOff)
    gameState.kickOffSetupFromTouchlines = false;
}

void GameStateProvider::reset(GameState& gameState)
{
  gameState = GameState();
  gameStateOverridden = false;
  manualPenaltyShootoutForOwnTeam = false;
  timeWhenStateNotAfterHalf = theFrameInfo.time;
  ballPositions.clear();
  lastBallAddedToBuffer = 0;
  illegalMotionInSetTimestamps.clear();
  illegalPositionTimestampsOwnTeam.clear();
  illegalPositionTimestampsOpponentTeam.clear();
  timeOfLastIntegratedGameControllerData = 0;
  timeWhenStateStartedBeforeWhistle = 0;
  timeWhenSwitchedToPlayingViaWhistle = 0;
  minWhistleTimestamp = 0;
}

bool GameStateProvider::checkForWhistle(unsigned from, bool useGameControllerData) const
{
  std::vector<const Whistle*> data;
  int numOfChannels = 0;
  int numOfMissingChannels = 0;
  size_t numOfVoters = 0;

  if(theWhistle.channelsUsedForWhistleDetection > 0)
  {
    ++numOfVoters;
    numOfChannels += theWhistle.channelsUsedForWhistleDetection;
    if(theWhistle.lastTimeWhistleDetected > from)
      data.emplace_back(&theWhistle);
  }

  for(const Teammate& teammate : theTeamData.teammates)
    if(teammate.theWhistle.channelsUsedForWhistleDetection > 0)
    {
      ++numOfVoters;
      if(teammate.theWhistle.lastTimeWhistleDetected > from)
      {
        numOfChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
        data.emplace_back(&teammate.theWhistle);
      }
      else
        numOfMissingChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
    }

  if(useGameControllerData)
  {
    size_t numOfPlayers = 0;
    const RoboCup::TeamInfo& ownTeam = theGameControllerData.teams[theGameControllerData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
    for(int i = 0; i < MAX_NUM_PLAYERS; ++i)
      if(ownTeam.players[i].penalty == PENALTY_NONE)
        ++numOfPlayers;
    const size_t numOfMissingPlayers = numOfPlayers - std::min(numOfPlayers, theTeamData.teammates.size() + 1);
    numOfVoters += numOfMissingPlayers;
    numOfMissingChannels += static_cast<int>(2 * numOfMissingPlayers); // Assume that unknown teammates use 2 channels
  }

  if(data.size() < minVotersForWhistle && data.size() < numOfVoters)
  {
    const size_t missing = minVotersForWhistle - data.size();
    numOfChannels += static_cast<int>(missing * numOfMissingChannels / (numOfVoters - data.size())); // Approximate number of missing channels
  }

  std::sort(data.begin(), data.end(),
            [](const Whistle* w1, const Whistle* w2) -> bool
  {
    return w1->lastTimeWhistleDetected < w2->lastTimeWhistleDetected;
  });

  for(size_t i = 0; i < data.size(); ++i)
  {
    float totalConfidence = 0.f;
    for(size_t j = i; j < data.size(); ++j)
    {
      if(static_cast<int>(data[j]->lastTimeWhistleDetected - data[i]->lastTimeWhistleDetected) > maxWhistleTimeDifference)
        break;

      totalConfidence += data[j]->confidenceOfLastWhistleDetection
                         * static_cast<float>(data[j]->channelsUsedForWhistleDetection);

      if(totalConfidence / numOfChannels > minWhistleAverageConfidence)
        return true;
    }
  }

  return false;
}

bool GameStateProvider::checkBallHasMoved(const GameState& gameState) const
{
  // Did anyone kick the ball?
  if(theMotionInfo.lastKickTimestamp > gameState.timeWhenStateStarted ||
     (Global::getSettings().scenario != "SharedAutonomyDefender" &&
      std::any_of(theTeamData.teammates.begin(), theTeamData.teammates.end(),
                 [&](const Teammate& teammate)
                 { return teammate.theIndirectKick.lastKickTimestamp > gameState.timeWhenStateStarted; })))
    return true;

  // Is the ball (significantly) closer to me than it could be, given that I am legally positioned?
  // -> theBallModel.timeWhenLastSeen == theFrameInfo.time can't be used because the ball model is (or at least can be) from the previous frame (USES).
  if(gameState.state == GameState::opponentKickOff &&
     theBallModel.timeWhenLastSeen > std::max(theFrameInfo.time - 500, gameState.timeWhenStateStarted) &&
     theBallModel.lastPerception.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - ballHasMovedCloseToRobotThreshold))
    return true;

  if((gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff) &&
     ballOutOfCenterCircleCounter >= ballOutOfCenterCircleCounterThreshold)
    return true;

  // Is there sufficient difference in recent (median filtered) ball positions?
  if(ballPositions.size() >= 6)
  {
    const auto medianOfThree = [](const Vector2f& a, const Vector2f& b, const Vector2f& c) -> const Vector2f&
    {
      const float aN = a.squaredNorm();
      const float bN = b.squaredNorm();
      const float cN = c.squaredNorm();

      if(aN >= bN)
      {
        if(cN >= aN)
          return a;
        else // c < a
          return bN >= cN ? b : c;
      }
      else // b > a
      {
        if(cN >= bN)
          return b;
        else // c < b
          return aN >= cN ? a : c;
      }
    };
    const std::size_t firstIndex = ballPositions.size() - 1;
    const Vector2f& start = medianOfThree(ballPositions.back(), ballPositions[firstIndex - 1], ballPositions[firstIndex - 2]);
    const Vector2f& end = medianOfThree(ballPositions.front(), ballPositions[1], ballPositions[2]);
    if((start - end).squaredNorm() > sqr(ballHasMovedTolerance) * std::max(1.f, std::min(start.squaredNorm(), end.squaredNorm()) / sqr(1000.f)))
      return true;
  }
  return false;
}

bool GameStateProvider::checkBallLegal(const GameState::State& gameState, const float confidenceIntervalToLegalPosition) const
{
  if(theBallModel.timeWhenLastSeen > theFrameInfo.time - 500
     && (GameState::isFreeKick(gameState) || gameState == GameState::ownPenaltyKick || gameState == GameState::opponentPenaltyKick) &&
     !GameState::isPushingFreeKick(gameState)) // only in these cases the ball position is constrained (kick of as well but the above code deals with that case)
  {
    const Vector2f globalBall = theRobotPose * theBallModel.estimate.position;
    //compute the covariance of the global ball model
    //start with the covariance of the local model
    Matrix2f cov = theBallModel.estimate.covariance;

    //rotate covariance matrix
    cov = Covariance::rotateCovarianceMatrix(cov, theRobotPose.rotation);

    //add covariance of the localization
    // todo: think about how to incorporate rotational uncertainty
    cov += theRobotPose.covariance.block<2, 2>(0, 0);

    COVARIANCE_ELLIPSES_2D("module:GameStateProvider:ballCovariance", cov, globalBall);

    //get the direction to the nearest legal position
    Vector2f direction(0.f, 0.f);
    if(GameState::isKickIn(gameState))
      direction << 0.f, (globalBall.y() > 0 ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline) - globalBall.y();
    else if(GameState::isGoalKick(gameState))
    {
      direction.y() = (globalBall.y() > 0 ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea) - globalBall.y();
      if(GameState::isForOpponentTeam(gameState))
        direction.x() = theFieldDimensions.xPosOpponentGoalArea - globalBall.x();
      else
        direction.x() = theFieldDimensions.xPosOwnGoalArea - globalBall.x();
    }
    else if(GameState::isCornerKick(gameState))
    {
      direction.y() = (globalBall.y() > 0 ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline) - globalBall.y();
      if(GameState::isForOpponentTeam(gameState))
        direction.x() = theFieldDimensions.xPosOwnGoalLine - globalBall.x();
      else
        direction.x() = theFieldDimensions.xPosOpponentGoalLine - globalBall.x();
    }
    else if(gameState == GameState::ownPenaltyKick)
    {
      direction = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0) - globalBall;
    }
    else if(gameState == GameState::opponentPenaltyKick)
    {
      direction = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0) - globalBall;
    }
    // ball is exactly on a legal position
    if(direction.squaredNorm() == 0.f)
      return true;

    ARROW("module:GameStateProvider:ballCovariance", globalBall.x(), globalBall.y(),
          (globalBall + direction).x(), (globalBall + direction).y(), 10, Drawings::PenStyle::solidPen,
          ColorRGBA::red);

    // construct covariance ellipse
    float a, b, angle;
    Covariance::errorEllipse(cov, a, b, angle, confidenceIntervalToLegalPosition);

    // compute radius in the previous calculated direction
    Angle theta(angle);
    theta += direction.angle();
    theta.normalize();

    const float radius = (a * b) / std::sqrt(sqr(a * std::sin(theta)) + sqr(b * std::cos(theta)));

    //if the ball is further from the next legal position than the radius + a placement tolerance it is likely that the ball was already played
    if(sqr(radius + ballPlacementTolerance) < direction.squaredNorm())
      return false;
  }
  return true;
}

bool GameStateProvider::checkForIllegalMotionInSetPenalty(unsigned from) const
{
  return std::any_of(illegalMotionInSetTimestamps.begin(), illegalMotionInSetTimestamps.end(), [from](unsigned timestamp)
  {
    return timestamp > from;
  });
}

void GameStateProvider::updateBallBuffer(const GameState& gameState)
{
  // Only buffer balls during states in which the ball should be stationary.
  if(gameState.state != GameState::ownKickOff && gameState.state != GameState::opponentKickOff &&
     gameState.state != GameState::ownPenaltyKick && gameState.state != GameState::opponentPenaltyKick &&
     !gameState.isFreeKick())
  {
    ballPositions.clear();
    lastBallUsedForCenterCircleCounter = lastBallAddedToBuffer = theBallModel.timeWhenLastSeen;
    ballOutOfCenterCircleCounter = 0;
    return;
  }

  if((gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff) && theBallModel.timeWhenLastSeen > lastBallUsedForCenterCircleCounter)
  {
    const Vector2f ballPosition = theRobotPose * theBallModel.estimate.position;
    if(ballPosition.squaredNorm() >= sqr(theFieldDimensions.centerCircleRadius + theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius + ballOutOfCenterCircleTolerance))
      ++ballOutOfCenterCircleCounter;
    else if(ballOutOfCenterCircleCounter)
      --ballOutOfCenterCircleCounter;
    lastBallUsedForCenterCircleCounter = theBallModel.timeWhenLastSeen + ballSaveInterval;
  }

  // Remove balls when the camera matrix is likely to be unstable.
  // If this should someday also work if the robot is walking, the ball buffer would need odometry updates.
  if(theMotionInfo.executedPhase != MotionPhase::stand || theIMUValueState.gyroValues.stableSinceTimestamp == theIMUValueState.timestamp)
    ballPositions.clear();
  else if(theBallModel.timeWhenLastSeen > lastBallAddedToBuffer + ballSaveInterval)
  {
    // Only add new balls every ballSaveInterval milliseconds.
    ballPositions.push_front(theBallModel.lastPerception);
    lastBallAddedToBuffer = theBallModel.timeWhenLastSeen;
  }
}

void GameStateProvider::updateIllegalMotionInSetTimestamps()
{
  // For each player, set the timestamp in the first frame in which the penalty is illegal motion in set.
  illegalMotionInSetTimestamps.resize(2 * MAX_NUM_PLAYERS, 0);
  for(unsigned int i = 0; i < 2; ++i)
    for(unsigned int j = 0; j < MAX_NUM_PLAYERS; ++j)
      if(theGameControllerData.teams[i].players[j].penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
        illegalMotionInSetTimestamps[i * MAX_NUM_PLAYERS + j] = 0;
      else if(illegalMotionInSetTimestamps[i * MAX_NUM_PLAYERS + j] == 0)
        illegalMotionInSetTimestamps[i * MAX_NUM_PLAYERS + j] = theFrameInfo.time;
}

void GameStateProvider::updateIllegalPosition()
{
  illegalPositionTimestampsOwnTeam.resize(MAX_NUM_PLAYERS, 0);
  illegalPositionTimestampsOpponentTeam.resize(MAX_NUM_PLAYERS, 0);
  unsigned ownTeam = theGameControllerData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1;
  unsigned opponentTeam = theGameControllerData.teams[0].teamNumber != Global::getSettings().teamNumber ? 0 : 1;
  for(unsigned int j = 0; j < MAX_NUM_PLAYERS; ++j)
  {
    if(theGameControllerData.teams[ownTeam].players[j].penalty != PENALTY_SPL_ILLEGAL_POSITION)
      illegalPositionTimestampsOwnTeam[j] = 0;
    else if(illegalPositionTimestampsOwnTeam[j] == 0)
      illegalPositionTimestampsOwnTeam[j] = theFrameInfo.time;

    if(theGameControllerData.teams[opponentTeam].players[j].penalty != PENALTY_SPL_ILLEGAL_POSITION)
      illegalPositionTimestampsOpponentTeam[j] = 0;
    else if(illegalPositionTimestampsOpponentTeam[j] == 0)
      illegalPositionTimestampsOpponentTeam[j] = theFrameInfo.time;
  }
}

GameState::Phase GameStateProvider::convertGameControllerDataToPhase(const GameControllerData& gameControllerData)
{
  return gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT ?
         GameState::penaltyShootout :
         (gameControllerData.firstHalf ?
          GameState::firstHalf :
          GameState::secondHalf);
}

GameState::State GameStateProvider::convertGameControllerDataToState(const GameControllerData& gameControllerData)
{
  const bool isKickingTeam = gameControllerData.kickingTeam == Global::getSettings().teamNumber;
  if(gameControllerData.gamePhase == GAME_PHASE_TIMEOUT)
  {
    ASSERT(gameControllerData.state == STATE_INITIAL);
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::timeout;
  }
  else if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    if(gameControllerData.state == STATE_INITIAL)
      return GameState::beforePenaltyShootout;
    else if(gameControllerData.state == STATE_SET)
      return isKickingTeam ? GameState::waitForOwnPenaltyShot : GameState::waitForOpponentPenaltyShot;
    else if(gameControllerData.state == STATE_PLAYING)
      return isKickingTeam ? GameState::ownPenaltyShot : GameState::opponentPenaltyShot;
    else if(gameControllerData.state == STATE_FINISHED)
      return isKickingTeam ? GameState::afterOwnPenaltyShot : GameState::afterOpponentPenaltyShot;
    else
      FAIL("Impossible game state.");
  }
  ASSERT(gameControllerData.gamePhase == GAME_PHASE_NORMAL);
  if(gameControllerData.state == STATE_INITIAL)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::beforeHalf;
  }
  else if(gameControllerData.state == STATE_STANDBY)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::standby;
  }
  else if(gameControllerData.state == STATE_READY)
  {
    if(gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
      return isKickingTeam ? GameState::setupOwnPenaltyKick : GameState::setupOpponentPenaltyKick;
    else
    {
      ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
      return isKickingTeam ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff;
    }
  }
  else if(gameControllerData.state == STATE_SET)
  {
    if(gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
      return isKickingTeam ? GameState::waitForOwnPenaltyKick : GameState::waitForOpponentPenaltyKick;
    else
    {
      ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
      return isKickingTeam ? GameState::waitForOwnKickOff : GameState::waitForOpponentKickOff;
    }
  }
  else if(gameControllerData.state == STATE_PLAYING)
  {
    if(gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
      return isKickingTeam ? GameState::ownPenaltyKick : GameState::opponentPenaltyKick;
    else if(gameControllerData.setPlay == SET_PLAY_PUSHING_FREE_KICK)
      return isKickingTeam ? GameState::ownPushingFreeKick : GameState::opponentPushingFreeKick;
    else if(gameControllerData.setPlay == SET_PLAY_KICK_IN)
      return isKickingTeam ? GameState::ownKickIn : GameState::opponentKickIn;
    else if(gameControllerData.setPlay == SET_PLAY_GOAL_KICK)
      return isKickingTeam ? GameState::ownGoalKick : GameState::opponentGoalKick;
    else if(gameControllerData.setPlay == SET_PLAY_CORNER_KICK)
      return isKickingTeam ? GameState::ownCornerKick : GameState::opponentCornerKick;
    else
    {
      ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
      if(gameControllerData.secondaryTime > 0)
      {
        // The GameController doesn't explicitly have a state for this in the network messages,
        // but a secondaryTime != 0 can only happen during kick-off. This only happens when the
        // GameController's 15s delay after the whistle ends early due to an event that could
        // not occur during SET.
        return isKickingTeam ? GameState::ownKickOff : GameState::opponentKickOff;
      }
      else
        return GameState::playing;
    }
  }
  else if(gameControllerData.state == STATE_FINISHED)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::afterHalf;
  }
  else
    FAIL("Impossible game state.");
  return GameState::playing;
}

GameState::PlayerState GameStateProvider::convertPenaltyToPlayerState(decltype(RoboCup::RobotInfo::penalty) penalty)
{
  switch(penalty)
  {
    case PENALTY_MANUAL:
      return GameState::penalizedManual;
    case PENALTY_SPL_ILLEGAL_BALL_CONTACT:
      return GameState::penalizedIllegalBallContact;
    case PENALTY_SPL_PLAYER_PUSHING:
      return GameState::penalizedPlayerPushing;
    case PENALTY_SPL_ILLEGAL_MOTION_IN_SET:
      return GameState::penalizedIllegalMotionInSet;
    case PENALTY_SPL_INACTIVE_PLAYER:
      return GameState::penalizedInactivePlayer;
    case PENALTY_SPL_ILLEGAL_POSITION:
      return GameState::penalizedIllegalPosition;
    case PENALTY_SPL_LEAVING_THE_FIELD:
      return GameState::penalizedLeavingTheField;
    case PENALTY_SPL_REQUEST_FOR_PICKUP:
      return GameState::penalizedRequestForPickup;
    case PENALTY_SPL_LOCAL_GAME_STUCK:
      return GameState::penalizedLocalGameStuck;
    case PENALTY_SPL_ILLEGAL_POSITION_IN_SET:
      return GameState::penalizedIllegalPositionInSet;
    case PENALTY_SPL_PLAYER_STANCE:
      return GameState::penalizedPlayerStance;
    case PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY:
      return GameState::penalizedIllegalMotionInStandby;
    case PENALTY_SUBSTITUTE:
      return GameState::substitute;
    case PENALTY_NONE:
      return GameState::active;
    default:
      FAIL("Unknown penalty.");
  }
  return GameState::active;
}
