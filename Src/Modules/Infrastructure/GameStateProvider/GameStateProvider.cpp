/**
 * @file GameStateProvider.cpp
 *
 * This file implements a module that provides a condensed representation of the game state.
 *
 * @author Arne Hasselbring
 */

#include "GameStateProvider.h"
#include "Debugging/Debugging.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <algorithm>

MAKE_MODULE(GameStateProvider, infrastructure);

void GameStateProvider::update(GameState& gameState)
{
  auto setToUnstiff = [this, &gameState]
  {
    reset(gameState);
  };

  auto setToCalibration = [this, &gameState]
  {
    reset(gameState);
    gameState.state = GameState::playing;
    gameState.timeWhenStateStarted = theFrameInfo.time;
    timeForWhistle = gameState.timeWhenStateStarted;
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
    setToActive();

  if(gameState.state != GameState::afterHalf)
    timeWhenStateNotAfterHalf = theFrameInfo.time;

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
         theBehaviorStatus.activity == BehaviorStatus::calibrationFinished)
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
        // If the game controller has not also changed state after the delay we had a false positiv whistle
        if(gameState.gameControllerActive && theGameControllerData.state != STATE_READY &&
           (theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > goalSignalDelay + gameControllerOperatorDelay ||
            (theGameControllerData.setPlay != SET_PLAY_NONE && theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > gameControllerOperatorDelay)))
        {
          gameStateOverridden = false;
          gameState.state = GameState::playing;
          gameState.timeWhenStateStarted = timeWhenStateStartedBeforeWhistle;
          timeForWhistle = theFrameInfo.time;
          gameState.timeWhenStateEnds = 0;
        }
        break;
      case GameState::ownKickOff:
      case GameState::opponentKickOff:
        // If we are still in set we had a false positive whistle
        if(checkForIllegalMotionInSetPenalty(gameState.timeWhenStateStarted) || (theGameControllerData.state == STATE_SET &&
           theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) > kickOffDuration + gameControllerOperatorDelay))
        {
          gameStateOverridden = false;
          gameState.state = gameState.state == GameState::ownKickOff ? GameState::waitForOwnKickOff : GameState::waitForOpponentKickOff;
          gameState.timeWhenStateStarted = timeWhenStateStartedBeforeWhistle;
          timeForWhistle = theFrameInfo.time;
          gameState.timeWhenStateEnds = 0;
        }
        break;
    }
  }

  if(gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff)
  {
    // A kick-off is over once either the ball has moved or the time has elapsed.
    if(checkBallHasMoved(gameState) || theFrameInfo.time >= gameState.timeWhenStateEnds)
    {
      gameStateOverridden = !theGameControllerData.isTrueData;
      gameState.state = GameState::playing;
      gameState.timeWhenStateStarted = theFrameInfo.time;
      timeForWhistle = theFrameInfo.time;
      gameState.timeWhenStateEnds = 0;
    }
  }

  // State transitions that are triggered by the whistle
  if(!gameState.gameControllerActive || !theGameControllerData.isTrueData)
  {
    switch(gameState.state)
    {
      case GameState::playing:
      case GameState::ownKickOff:
      case GameState::opponentKickOff:
      case GameState::ownPenaltyKick:
      case GameState::opponentPenaltyKick:
      case GameState::ownPushingFreeKick:
      case GameState::opponentPushingFreeKick:
      case GameState::ownKickIn:
      case GameState::opponentKickIn:
      case GameState::ownGoalKick:
      case GameState::opponentGoalKick:
      case GameState::ownCornerKick:
      case GameState::opponentCornerKick:
        // The referee whistles because of goal (after a CoolDown time to avoid detecting the same whistle twice)
        if(checkWhistleForGoal &&
           checkForWhistle(timeForWhistle
                           + ((gameState.state == GameState::ownPenaltyKick || gameState.state == GameState::opponentPenaltyKick)
                              ? ignoreWhistleAfterPenaltyKick : ignoreWhistleAfterKickOff)) && (theBallInGoal.timeSinceLastInGoal <= acceptBallInGoalDelay || !checkBallForGoal))
        {
          // Remember start of previous state in case to whistle detection turns out to be wrong
          timeWhenStateStartedBeforeWhistle = gameState.timeWhenStateStarted;
          gameStateOverridden = true;
          gameState.state = theBallInGoal.inOwnGoal ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff;
          gameState.kickOffSetupFromSidelines = false;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          timeForWhistle = theFrameInfo.time;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
          ++(theBallInGoal.inOwnGoal ? gameState.opponentTeam : gameState.ownTeam).score;
        }
        break;
      case GameState::waitForOwnKickOff:
      case GameState::waitForOpponentKickOff:
      case GameState::waitForOwnPenaltyKick:
      case GameState::waitForOpponentPenaltyKick:
      case GameState::waitForOwnPenaltyShot:
      case GameState::waitForOpponentPenaltyShot:
        if(checkForWhistle(timeForWhistle))
        {
          // Remember start of previous state in case to whistle detection turns out to be wrong
          timeWhenStateStartedBeforeWhistle = gameState.timeWhenStateStarted;
          gameStateOverridden = true;
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
          timeForWhistle = theFrameInfo.time;
          gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffDuration;
        }
        break;
    }
  }

  if(useGameControllerData)
  {
    // The index of the own team within theGameControllerData.
    const std::size_t ownTeamIndex = theGameControllerData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1;
    // Everything else would be a bug in the GameControllerDataProvider:
    ASSERT(theGameControllerData.teams[ownTeamIndex].teamNumber == Global::getSettings().teamNumber);

    // The game phase is always reported truthfully by the GameController.
    // See below for timeWhenPhaseEnds.
    gameState.phase = convertGameControllerDataToPhase(theGameControllerData);

    auto gameControllerState = convertGameControllerDataToState(theGameControllerData);
    // The game state is not overridden anymore if both states match or the GC sends a state that is always true (i.e. neither SET nor PLAYING).
    gameStateOverridden &= gameState.state != gameControllerState;
    gameStateOverridden &= GameState::isSet(gameControllerState) || GameState::isPlaying(gameControllerState);
    // Free Kicks are mostly true (so we stop overriding), but not if we guess ready (because it might be that the GC still sends a free kick when a goal is scored).
    gameStateOverridden &= !GameState::isFreeKick(gameControllerState) || gameState.isReady();
    if(!gameStateOverridden)
    {
      // The GameController message does not have the concept of the kick-off state. Therefore, if the GameController sends true data
      // (such as in the simulator) and the whistle isn't used, the state is created here at the transition from SET to PLAYING.
      if(theGameControllerData.isTrueData && gameControllerState == GameState::playing &&
         (gameState.state == GameState::waitForOwnKickOff || gameState.state == GameState::waitForOpponentKickOff))
      {
        gameControllerState = gameState.state == GameState::waitForOwnKickOff ? GameState::ownKickOff : GameState::opponentKickOff;
        gameStateOverridden = true;
      }
      if(gameControllerState != gameState.state)
      {
        gameState.timeWhenStateStarted = theFrameInfo.time;
        timeForWhistle = theFrameInfo.time;
        switch(gameControllerState)
        {
          case GameState::setupOwnKickOff:
          case GameState::setupOpponentKickOff:
            // Switching to READY while changing the score means we haven't heard the whistle after a goal.
            // In that case, the state actually began earlier.
            if(!theGameControllerData.isTrueData &&
               (theGameControllerData.teams[ownTeamIndex].score != gameState.ownTeam.score ||
                theGameControllerData.teams[1 - ownTeamIndex].score != gameState.opponentTeam.score))
              gameState.timeWhenStateStarted -= goalSignalDelay;
            gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
            break;
          case GameState::ownKickOff:
          case GameState::opponentKickOff:
            // This can only happen as part of the case above. \c convertGameControllerDataToState can never return those states.
            ASSERT(theGameControllerData.isTrueData);
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
      // kickOffSetupFromSidelines: We are switching from some INITIAL state to READY and we already integrated a GameController packet recently.
      if((gameControllerState == GameState::setupOwnKickOff || gameControllerState == GameState::setupOpponentKickOff) &&
         gameState.isInitial() && theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) < gameControllerTimeout)
        gameState.kickOffSetupFromSidelines = true;
      gameState.state = gameControllerState;
    }
    else if(gameState.state == gameControllerState)
      gameStateOverridden = false;

    auto fillTeam = [&](const RoboCup::TeamInfo& teamInfo, bool speculatingGoalFor, GameState::Team& team)
    {
      static_assert(std::tuple_size<decltype(team.playerStates)>::value == MAX_NUM_PLAYERS);
      for(std::size_t i = 0; i < team.playerStates.size(); ++i)
        team.playerStates[i] = convertPenaltyToPlayerState(teamInfo.players[i].penalty);
      team.number = teamInfo.teamNumber;
      team.color = static_cast<GameState::Team::Color>(teamInfo.teamColor);
      team.score = teamInfo.score + (speculatingGoalFor ? 1 : 0);
      team.messageBudget = teamInfo.messageBudget;
    };

    fillTeam(theGameControllerData.teams[ownTeamIndex], gameState.state == GameState::setupOpponentKickOff && gameControllerState != gameState.state, gameState.ownTeam);
    fillTeam(theGameControllerData.teams[1 - ownTeamIndex], gameState.state == GameState::setupOwnKickOff && gameControllerState != gameState.state, gameState.opponentTeam);

    const auto gameControllerPlayerState = convertPenaltyToPlayerState(theGameControllerData.teams[ownTeamIndex].players[Global::getSettings().playerNumber - 1].penalty);
    if(gameControllerPlayerState != gameState.playerState)
    {
      // If this is the first GameController packet for some time, it might be that the robot actually has had the state for a longer time.
      if(theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) >= gameControllerTimeout)
        gameState.timeWhenPlayerStateStarted = 0;
      else
        gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
    }
    gameState.playerState = gameControllerPlayerState;

    timeOfLastIntegratedGameControllerData = theGameControllerData.timeLastPacketReceived;
  }
  else if(!gameState.gameControllerActive)
  {
    // Button interface (only when we haven't received from a GameController recently):
    // - chest button to switch between manual penalty and playing / penalty shoot-out
    // - foot bumpers
    //   - left foot for own team color
    //   - right foot to switch between penalty shoot-out and normal mode
    if(!ignoreChestButton && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
    {
      if(gameState.playerState == GameState::active)
        gameState.playerState = GameState::penalizedManual;
      else
      {
        gameState.playerState = GameState::active;
        if(gameState.phase == GameState::penaltyShootout)
        {
          gameState.state = manualPenaltyShootoutForOwnTeam ? GameState::ownPenaltyShot : GameState::opponentPenaltyShot;
          gameState.timeWhenStateStarted = theFrameInfo.time - playingSignalDelay;
          timeForWhistle = gameState.timeWhenStateStarted;
          gameState.timeWhenPhaseEnds = gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickDuration;
        }
        else
        {
          gameState.state = GameState::playing;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          timeForWhistle = gameState.timeWhenStateStarted;
          gameState.timeWhenStateEnds = 0;
        }
      }
      gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
    }

    if(gameState.isInitial() && gameState.playerState == GameState::active)
    {
      if(theEnhancedKeyStates.hitStreak[KeyStates::lFootLeft] == 1)
        gameState.ownTeam.color = static_cast<GameState::Team::Color>((static_cast<unsigned>(gameState.ownTeam.color) + 1) % GameState::Team::Color::numOfTeamColors);

      if(theEnhancedKeyStates.hitStreak[KeyStates::rFootRight] == 1)
      {
        if(gameState.phase != GameState::penaltyShootout)
        {
          gameState.phase = GameState::penaltyShootout;
          gameState.timeWhenPhaseEnds = 0;
          gameState.state = GameState::beforePenaltyShootout;
          gameState.timeWhenStateStarted = theFrameInfo.time;
          timeForWhistle = gameState.timeWhenStateStarted;
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
          timeForWhistle = gameState.timeWhenStateStarted;
          gameState.timeWhenStateEnds = 0;
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
    gameState.kickOffSetupFromSidelines = false;
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
  timeOfLastIntegratedGameControllerData = 0;
}

bool GameStateProvider::checkForWhistle(unsigned timeOfLastStateChange) const
{
  std::vector<const Whistle*> data;
  int numOfChannels = 0;

  if(theWhistle.channelsUsedForWhistleDetection > 0)
  {
    numOfChannels += theWhistle.channelsUsedForWhistleDetection;
    if(theWhistle.lastTimeWhistleDetected > timeOfLastStateChange)
      data.emplace_back(&theWhistle);
  }

  for(const Teammate& teammate : theTeamData.teammates)
    if(teammate.theWhistle.channelsUsedForWhistleDetection > 0)
    {
      numOfChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
      if(teammate.theWhistle.lastTimeWhistleDetected > timeOfLastStateChange)
        data.emplace_back(&teammate.theWhistle);
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
  // Did I kick the ball?
  if(theMotionInfo.lastKickTimestamp > gameState.timeWhenStateStarted)
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

bool GameStateProvider::checkForIllegalMotionInSetPenalty(unsigned timeOfLastStateChange) const
{
  return std::any_of(illegalMotionInSetTimestamps.begin(), illegalMotionInSetTimestamps.end(), [timeOfLastStateChange](unsigned timestamp)
  {
    return timestamp > timeOfLastStateChange;
  });
}

void GameStateProvider::updateBallBuffer(const GameState& gameState)
{
  // Only buffer balls during states in which the ball should be stationary.
  if(gameState.state != GameState::ownKickOff && gameState.state != GameState::opponentKickOff &&
     gameState.state != GameState::ownPenaltyKick && gameState.state != GameState::opponentPenaltyKick)
  {
    ballPositions.clear();
    lastBallUsedForCenterCircleCounter = lastBallAddedToBuffer = theBallModel.timeWhenLastSeen;
    ballOutOfCenterCircleCounter = 0;
    return;
  }

  if(gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff)
  {
    const bool useOwnBallModel = theBallModel.timeWhenLastSeen > lastBallUsedForCenterCircleCounter;
    if(useOwnBallModel || theTeammatesBallModel.isValid)
    {
      const Vector2f ballPosition = useOwnBallModel ? theRobotPose * theBallModel.estimate.position : theTeammatesBallModel.position;
      if(ballPosition.squaredNorm() >= sqr(theFieldDimensions.centerCircleRadius + theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius + ballOutOfCenterCircleTolerance))
        ++ballOutOfCenterCircleCounter;
      else if(ballOutOfCenterCircleCounter)
        --ballOutOfCenterCircleCounter;
      lastBallUsedForCenterCircleCounter = (useOwnBallModel ? theBallModel.timeWhenLastSeen : theFrameInfo.time) + ballSaveInterval;
    }
  }

  // Remove balls when the camera matrix is likely to be unstable.
  // If this should someday also work if the robot is walking, the ball buffer would need odometry updates.
  if(theMotionInfo.executedPhase != MotionPhase::stand || theGyroState.deviation.y() > maxGyroDeviationToDetectMovingBalls)
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
    // The following four cases are needed if set plays occur during the first 15 seconds of playing.
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
    case PENALTY_SUBSTITUTE:
      return GameState::substitute;
    case PENALTY_NONE:
      return GameState::active;
    default:
      FAIL("Unknown penalty.");
  }
  return GameState::active;
}
