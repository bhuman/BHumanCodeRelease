/**
 * @file GameState.h
 *
 * This file declares a representation of the game state in a way that is
 * useful for behaviors etc.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Framework/Settings.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <algorithm>
#include <array>

STREAMABLE(GameState,
{
  ENUM(State,
  {,
    beforeHalf, /**< The half hasn't started yet. */
    standby, /**< Wait for the signal to walk in from the touchlines. */
    afterHalf, /**< The half is over. */
    timeout, /**< A team or the referee has taken a timeout. */

    playing, /**< Regular play, no special privileges or restrictions (except for potential illegal goals). */

    setupOwnKickOff, /**< The ready phase before an own kick-off. */
    setupOpponentKickOff, /**< The ready phase before an opponent kick-off. */
    waitForOwnKickOff, /**< The set phase before an own kick-off. */
    waitForOpponentKickOff, /**< The set phase before an opponent kick-off. */
    ownKickOff, /**< The phase of an own kick-off in which the opponent must not enter the center circle. */
    opponentKickOff, /**< The phase of an opponent kick-off in which we must not enter the center circle. */

    setupOwnPenaltyKick, /**< The ready phase before an own penalty kick. */
    setupOpponentPenaltyKick, /**< The ready phase before an opponent penalty kick. */
    waitForOwnPenaltyKick, /**< The set phase before an own penalty kick. */
    waitForOpponentPenaltyKick, /**< The set phase before an opponent penalty kick. */
    ownPenaltyKick, /**< The phase of an own penalty kick in which the goalkeeper must stay on its line and nobody is allowed to enter the penalty area. */
    opponentPenaltyKick, /**< The phase of an opponent penalty kick in which the goalkeeper must stay on its line and nobody is allowed to enter the penalty area. */

    ownPushingFreeKick, /**< A pushing free kick for the own team. */
    opponentPushingFreeKick, /**< A pushing free kick for the opponent team. */
    ownKickIn, /**< A kick-in for the own team. */
    opponentKickIn, /**< A kick-in for the opponent team. */
    ownGoalKick, /**< A goal kick for the own team. */
    opponentGoalKick, /**< A goal kick for the opponent team. */
    ownCornerKick, /**< A corner kick for the own team. */
    opponentCornerKick, /**< A corner kick for the opponent team. */

    beforePenaltyShootout, /**< A penalty shootout will begin. */
    waitForOwnPenaltyShot, /**< The set phase before an own penalty kick in a penalty shootout. */
    waitForOpponentPenaltyShot, /**< The set phase before an opponent penalty kick in a penalty shootout. */
    ownPenaltyShot, /**< An own penalty kick in a penalty shootout. */
    opponentPenaltyShot, /**< An opponent penalty kick in a penalty shootout. */
    afterOwnPenaltyShot, /**< The phase after an own penalty kick in a penalty shootout. */
    afterOpponentPenaltyShot, /**< The phase after on opponent penalty kick in a penalty shootout. */
  });

  ENUM(Phase,
  {,
    firstHalf, /**< The first half of a game. */
    secondHalf, /**< The second half of a game. */
    penaltyShootout, /**< A penalty shoot-out. */
  });

  ENUM(PlayerState,
  {,
    unstiff, /**< The robot is unstiff. */
    calibration, /**< The robot is in calibration mode. */
    penalizedManual, /**< The robot has been penalized using the chest button (no GameController is active). */
    penalizedIllegalBallContact, /**< The robot is penalized for an illegal ball contact (at the touchline, at least 45s). */
    penalizedPlayerPushing, /**< The robot is penalized for pushing (at the touchline, at least 45s). */
    penalizedIllegalMotionInSet, /**< The robot is penalized for illegal motion in set (in place, 15s). */
    penalizedInactivePlayer, /**< The robot is penalized for being inactive / fallen (at the touchline, 45s). */
    penalizedIllegalPosition, /**< The robot is penalized for being in an illegal position (at the touchline, at least 45s). */
    penalizedLeavingTheField, /**< The robot is penalized for leaving the field (at the touchline, at least 45s). */
    penalizedRequestForPickup, /**< The robot is picked up. */
    penalizedLocalGameStuck, /**< The robot is penalized for causing a local game stuck (at the touchline, 45s). */
    penalizedIllegalPositionInSet, /**< The robot is penalized for an illegal position in set (at the touchline, 15s). */
    penalizedPlayerStance, /**< The robot is penalized for an illegal stance (at the touchline, at least 45s). */
    penalizedIllegalMotionInStandby, /**< The robot is penalized for illegal motion in standby (in place, 15s). */
    substitute, /**< The robot is a substitute. */
    active, /**< The robot is playing according to the global game state. */
  });

  ENUM(CompetitionPhase,
  {,
    roundRobin, /**< Preliminary games, the clock is not stopped in ready and set. */
    playOff, /**< Final games. The clock is stopped in ready and set. */
  });

  STREAMABLE(Team,
  {
    using Color = Settings::TeamColor;

    /**
     * Checks whether a given player is the goalkeeper in a team.
     * @param playerNumber The player number.
     * @return Whether the player has goalkeeper privileges.
     */
    bool isGoalkeeper(int playerNumber) const
    {
      return playerNumber == goalkeeperNumber;
    }

    /**
     * Returns the number of players on the team that are active.
     * @return The number of players ...
     */
    unsigned int numOfActivePlayers() const
    {
      return static_cast<unsigned int>(std::count_if(playerStates.begin(), playerStates.end(), [](const PlayerState playerState){return playerState == active;}));
    }

    /**
     * Returns the jersey color of a given player in a team.
     * @param playerNumber The player number.
     * @return The jersey color of the given player.
     */
    Color color(int playerNumber) const
    {
      return isGoalkeeper(playerNumber) ? goalkeeperColor : fieldPlayerColor;
    },

    (std::array<PlayerState, Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1>)({}) playerStates, /**< The states of the players, indexed by jersey number - Settings::lowestValidPlayerNumber. */
    (int)(0) number, /**< The team number in the GameController. */
    (Color)(Color::black) fieldPlayerColor, /**< The jersey color of field players. */
    (Color)(Color::blue) goalkeeperColor, /**< The jersey color of the goalkeeper. */
    (int)(1) goalkeeperNumber, /**< The player number of the goalkeeper. */
    (unsigned int)(0u) score, /**< The current score. */
    (unsigned int)(1200u) messageBudget, /**< The remaining message budget. */
  });

  /** Constructor. */
  GameState();

  /** Draw the representation. */
  void draw() const;

  static bool isInitial(State state, bool orStandby = true)
  {
    return state == beforeHalf ||
           (orStandby && state == standby) ||
           state == timeout ||
           state == beforePenaltyShootout;
  }

  bool isInitial(bool orStandby = true) const
  {
    return isInitial(state, orStandby);
  }

  static bool isReady(State state)
  {
    return state == setupOwnKickOff ||
           state == setupOpponentKickOff ||
           state == setupOwnPenaltyKick ||
           state == setupOpponentPenaltyKick;
  }

  bool isReady() const
  {
    return isReady(state);
  }

  static bool isSet(State state)
  {
    return state == waitForOwnKickOff ||
           state == waitForOpponentKickOff ||
           state == waitForOwnPenaltyKick ||
           state == waitForOpponentPenaltyKick ||
           state == waitForOwnPenaltyShot ||
           state == waitForOpponentPenaltyShot;
  }

  bool isSet() const
  {
    return isSet(state);
  }

  static bool isPlaying(State state)
  {
    return state == playing ||
           state == ownKickOff ||
           state == opponentKickOff ||
           state == ownPenaltyKick ||
           state == opponentPenaltyKick ||
           state == ownPushingFreeKick ||
           state == opponentPushingFreeKick ||
           state == ownKickIn ||
           state == opponentKickIn ||
           state == ownGoalKick ||
           state == opponentGoalKick ||
           state == ownCornerKick ||
           state == opponentCornerKick ||
           state == ownPenaltyShot ||
           state == opponentPenaltyShot;
  }

  bool isPlaying() const
  {
    return isPlaying(state);
  }

  static bool isFinished(State state)
  {
    return state == afterHalf ||
           state == afterOwnPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isFinished() const
  {
    return isFinished(state);
  }

  static bool isStopped(State state)
  {
    return state == beforeHalf ||
           state == standby ||
           state == afterHalf ||
           state == timeout ||
           state == beforePenaltyShootout ||
           state == afterOwnPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isStopped() const
  {
    return isStopped(state);
  }

  static bool isKickOff(State state)
  {
    return state == setupOwnKickOff ||
           state == setupOpponentKickOff ||
           state == waitForOwnKickOff ||
           state == waitForOpponentKickOff ||
           state == ownKickOff ||
           state == opponentKickOff;
  }

  bool isKickOff() const
  {
    return isKickOff(state);
  }

  static bool isPenaltyKick(State state)
  {
    return state == setupOwnPenaltyKick ||
           state == setupOpponentPenaltyKick ||
           state == waitForOwnPenaltyKick ||
           state == waitForOpponentPenaltyKick ||
           state == ownPenaltyKick ||
           state == opponentPenaltyKick;
  }

  bool isPenaltyKick() const
  {
    return isPenaltyKick(state);
  }

  static bool isFreeKick(State state)
  {
    return state == ownPushingFreeKick ||
           state == opponentPushingFreeKick ||
           state == ownKickIn ||
           state == opponentKickIn ||
           state == ownGoalKick ||
           state == opponentGoalKick ||
           state == ownCornerKick ||
           state == opponentCornerKick;
  }

  bool isFreeKick() const
  {
    return isFreeKick(state);
  }

  static bool isPushingFreeKick(State state)
  {
    return state == ownPushingFreeKick ||
           state == opponentPushingFreeKick;
  }

  bool isPushingFreeKick() const
  {
    return isPushingFreeKick(state);
  }

  static bool isKickIn(State state)
  {
    return state == ownKickIn ||
           state == opponentKickIn;
  }

  bool isKickIn() const
  {
    return isKickIn(state);
  }

  static bool isGoalKick(State state)
  {
    return state == ownGoalKick ||
           state == opponentGoalKick;
  }

  bool isGoalKick() const
  {
    return isGoalKick(state);
  }

  static bool isCornerKick(State state)
  {
    return state == ownCornerKick ||
           state == opponentCornerKick;
  }

  bool isCornerKick() const
  {
    return isCornerKick(state);
  }

  static bool isPenaltyShootout(State state)
  {
    return state == beforePenaltyShootout ||
           state == waitForOwnPenaltyShot ||
           state == waitForOpponentPenaltyShot ||
           state == ownPenaltyShot ||
           state == opponentPenaltyShot ||
           state == afterOwnPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isPenaltyShootout() const
  {
    return isPenaltyShootout(state);
  }

  static bool isForOwnTeam(State state)
  {
    return state == setupOwnKickOff ||
           state == waitForOwnKickOff ||
           state == ownKickOff ||
           state == setupOwnPenaltyKick ||
           state == waitForOwnPenaltyKick ||
           state == ownPenaltyKick ||
           state == ownPushingFreeKick ||
           state == ownKickIn ||
           state == ownGoalKick ||
           state == ownCornerKick ||
           state == waitForOwnPenaltyShot ||
           state == ownPenaltyShot ||
           state == afterOwnPenaltyShot;
  }

  bool isForOwnTeam() const
  {
    return isForOwnTeam(state);
  }

  static bool isForOpponentTeam(State state)
  {
    return state == setupOpponentKickOff ||
           state == waitForOpponentKickOff ||
           state == opponentKickOff ||
           state == setupOpponentPenaltyKick ||
           state == waitForOpponentPenaltyKick ||
           state == opponentPenaltyKick ||
           state == opponentPushingFreeKick ||
           state == opponentKickIn ||
           state == opponentGoalKick ||
           state == opponentCornerKick ||
           state == waitForOpponentPenaltyShot ||
           state == opponentPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isForOpponentTeam() const
  {
    return isForOpponentTeam(state);
  }

  static bool isPenalized(PlayerState state)
  {
    return state == penalizedManual ||
           state == penalizedIllegalBallContact ||
           state == penalizedPlayerPushing ||
           state == penalizedIllegalMotionInSet ||
           state == penalizedInactivePlayer ||
           state == penalizedIllegalPosition ||
           state == penalizedLeavingTheField ||
           state == penalizedRequestForPickup ||
           state == penalizedLocalGameStuck ||
           state == penalizedIllegalPositionInSet ||
           state == penalizedIllegalMotionInStandby ||
           state == penalizedPlayerStance ||
           state == substitute;
  }

  bool isPenalized() const
  {
    return isPenalized(playerState);
  }

  bool isGoalkeeper() const
  {
    return ownTeam.isGoalkeeper(playerNumber);
  }

  Team::Color color() const
  {
    return ownTeam.color(playerNumber);
  },

  (Phase)(firstHalf) phase, /**< The current phase of the game. */
  (unsigned)(0) timeWhenPhaseEnds, /**< Time when the current phase is expected to end. */
  (State)(beforeHalf) state, /**< The current state of the game. */
  (unsigned)(0) timeWhenStateStarted, /**< Time when the current state started. */
  (unsigned)(0) timeWhenStateEnds, /**< Time when the current state is expected to end. Only valid for some states! */
  (bool)(false) kickOffSetupFromTouchlines, /**< During a kick-off setup: Is it from the touchlines (i.e. start of half / after timeout as opposed to after a goal / global game stuck)? */
  (Team) ownTeam, /**< The state of the own team. */
  (Team) opponentTeam, /**< The state of the opponent team. */
  (int)(0) playerNumber, /**< The jersey number of this player. */
  (PlayerState)(unstiff) playerState, /**< The current state of this player. */
  (unsigned)(0) timeWhenPlayerStateStarted, /**< Time when the current player state started. */
  (bool)(false) gameControllerActive, /**< Whether a GameController is active. */
  (bool)(true) leftHandTeam, /**< Whether the own team defends the left goal from the GameController's PoV. */
  (CompetitionPhase)(roundRobin) competitionPhase, /**< The phase of the tournament we are in. */
  (bool)(false) whistled, /**< State was switched due to a whistle. */
  (unsigned)(0) timeWhenPenaltyEnds, /**< Time when the own penalty is expected to end (0 -> not penalized). */
});

STREAMABLE(ExtendedGameState,
{
  bool wasInitial(bool orStandby = true) const
  {
    return GameState::isInitial(stateLastFrame, orStandby);
  }

  bool wasReady() const
  {
    return GameState::isReady(stateLastFrame);
  }

  bool wasSet() const
  {
    return GameState::isSet(stateLastFrame);
  }

  bool wasPlaying() const
  {
    return GameState::isPlaying(stateLastFrame);
  }

  bool wasFinished() const
  {
    return GameState::isFinished(stateLastFrame);
  }

  bool wasPenalized() const
  {
    return GameState::isPenalized(playerStateLastFrame);
  },

  (GameState::State)(GameState::beforeHalf) stateLastFrame, /**< The state of the game in the last frame. */
  (GameState::PlayerState)(GameState::unstiff) playerStateLastFrame, /**< The state of this player in the last frame. */
  (unsigned)(1200u) messageBudgetLastFrame, /**< Message budget of the own team in the last frame. */
  (bool)(false) returnFromGameControllerPenalty, /**< Is the robot returning from a GameController penalty this frame? */
  (bool)(false) returnFromManualPenalty, /**< Is the robot returning from a manual penalty this frame? */

  (std::array<unsigned, GameState::numOfStates>)({}) timeWhenStateStarted, /**< For each state: Time when this state started last. */
  (std::array<unsigned, GameState::numOfPlayerStates>)({}) timeWhenPlayerStateStarted, /**< For each player state: Time when this state started last. */
});
