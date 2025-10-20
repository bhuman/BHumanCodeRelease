/**
 * @file SimulatedNao/GameController.h
 * This file declares a class that simulates a console-based GameController.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/GameControllerData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Communication/TeamMessageContainer.h"
#include "Framework/Settings.h"
#include "Math/Pose2f.h"
#include "Streaming/Enum.h"
#include <SimRobot.h>
#include <set>
#include <string>

class SimulatedRobot;
class TeamMessageChannel; // Importing the header would create Windows.h/WinSock2.h conflicts on Windows

/**
 * The class simulates a console-based GameController.
 */
class GameController
{
public:
  ENUM(AutomaticReferee,
  {,
    trueGameState,
    placeBall,
    placePlayers,
    switchToSet,
    switchToPlaying,
    switchToFinished,
    ballOut,
    freeKickComplete,
    clearBall,
    penalizeLeavingTheField,
    penalizeIllegalPosition,
    penalizeIllegalPositionInSet,
    unpenalize,
    kickOffDelay,
    directGoals,
  });

  unsigned automatic = ~0u; /**< Which automatic features are active? */

  ENUM(Penalty,
  {,
    none,
    illegalBallContact,
    playerPushing,
    illegalMotionInSet,
    inactivePlayer,
    illegalPosition,
    leavingTheField,
    requestForPickup,
    localGameStuck,
    illegalPositionInSet,
    playerStance,
    substitute,
    manual,
    foul,
    penaltyKick,
  });
  static ENUM_NUM_OF_ALIAS(Penalty, numOfPenalties); /**< Correct typo. */

  struct TeamInfo
  {
    uint8_t number = 0;
    Settings::TeamColor fieldPlayerColor = static_cast<Settings::TeamColor>(-1);
    Settings::TeamColor goalkeeperColor = static_cast<Settings::TeamColor>(-1);
  };

private:
  struct Robot
  {
    SimulatedRobot* simulatedRobot = nullptr;
    RoboCup::RobotInfo* info = nullptr;
    unsigned timeWhenPenalized = 0;
    unsigned timeWhenBallNotStuckBetweenLegs = 0;
    float ownGoalAreaMargin = -1.f;
    float ownPenaltyAreaMargin = -1.f;
    float opponentPenaltyAreaMargin = -1.f;
    uint8_t lastPenalty = PENALTY_NONE;
    Pose2f lastPose;
  };

  enum KickOffReason
  {
    kickOffReasonHalf,
    kickOffReasonPenalty,
    kickOffReasonGoal,
    kickOffReasonGlobalGameStuck
  };

  static const int numOfRobots = 2 * MAX_NUM_PLAYERS;
  static const int halfTime = 600;
  static const int readyTime = 45;
  static const int penaltyKickReadyTime = 30;
  static const int kickOffTime = 10;
  static const int freeKickTime = 30;
  static const int penaltyShotTime = 30;
  static const int delayedSwitchToPlaying = 15;
  static const int delayedSwitchAfterGoal = 15;
  static const int refereeStandbyDelay = 5; /**< How long before the referee appears in standby state (in s)? */
  static const int refereeStandbyTime = 2; /**< How long is the referee visible in standby state (in s)? */
  static const int refereeSetPlayTime = 5; /**< How long is the referee visible during a set play (in s)? */
  float footLength = 120.f; /**< foot length for position check. */
  float dropHeight = 350.f; /**< height at which robots are placed so the fall a little bit and recognize it. */
  float penaltyPlacementDistance = 400.f; /**< The distance between robots when multiple are penalized. */
  Pose2f lastBallContactPose; /**< Position where the last ball contact of a robot took place, orientation is toward opponent goal (0/180 degrees). */
  unsigned lastBallContactTime = 0;
  SimRobot::Object* lastBallContactRobots[2] = {nullptr, nullptr}; /**< Last robot touching the ball per team. */
  SimRobot::Object* refereeObject = nullptr; /**< The visual representation of the referee. */
  FieldDimensions fieldDimensions;
  BallSpecification ballSpecification;
  GameControllerData gameControllerData;
  Whistle whistle;
  uint8_t lastState = STATE_INITIAL;
  uint8_t kickingTeamBeforeGoal = 0;
  KickOffReason kickOffReason = kickOffReasonHalf;
  unsigned timeBeforeCurrentState = 0;
  unsigned timeWhenLastRobotMoved = 0;
  unsigned timeWhenStateBegan = 0;
  unsigned timeWhenSetPlayBegan = 0;
  Robot robots[numOfRobots];
  int ballContacts[2];
  int testRuns = 0;
  int testRunCounter = 0;

  TeamMessageContainer inTeamMessage;
  TeamMessageContainer outTeamMessage;
  TeamMessageChannel* theTeamMessageChannel;

  /** enum which declares the different types of balls leaving the field */
  enum BallOut
  {
    notOut,
    goalBySecondTeam,
    goalByFirstTeam,
    kickInSecondTeam,
    kickInFirstTeam,
    cornerKickSecondTeam,
    cornerKickFirstTeam,
    goalKickSecondTeam,
    goalKickFirstTeam
  };

  /** The different types of referee signals. */
  enum RefereeSignal
  {
    noSignal,
    up,
    left,
    right
  };

public:
  GameController();
  ~GameController();

  /**
   * Sets the team information that is not available at construction time.
   * @param teamInfos The settings of the playing teams.
   */
  void setTeamInfos(const std::array<TeamInfo, 2>& teamInfos);

  /** Load ball specification after the search path has been filled. */
  void loadBallSpecification();

  /**
   * Each simulated robot must be registered.
   * @param robot The number of the robot [0 ... numOfRobots-1].
   * @param simulatedRobot The simulation interface of that robot.
   */
  void registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot);

  /**
   * The visual representation of the referee must be registered.
   * @param referee The SimRobot object for the referee. Can be
   *                nullptr if there is none.
   */
  void registerReferee(SimRobot::Object* referee) {refereeObject = referee;}

  bool initial();
  bool standby();
  virtual bool ready(); // overridden by TestGameController
  bool set();
  virtual bool playing(); // overridden by TestGameController
  virtual bool finished(); // overridden by TestGameController
  bool competitionPhasePlayoff();
  bool competitionPhaseRoundrobin();
  bool competitionTypeChampionsCup();
  bool competitionTypeChallengeShield();
  bool globalGameStuck();
  virtual bool goal(int side);
  bool goalKick(int side);
  bool pushingFreeKick(int side);
  bool cornerKick(int side);
  bool kickIn(int side);
  bool teamPenaltyKick(int side);
  bool kickOff(int side);
  bool dropBall();
  bool setHalf(int half); // 1, 2
  bool gamePhasePenaltyshoot();
  bool gamePhaseNormal();
  virtual bool penalty(int robot, Penalty penalty); // overridden by TestGameController

  /** Executes the automatic referee. */
  void update();

  /**
   * Proclaims which robot touched the ball at last
   * @param robot The robot
   */
  void setLastBallContactRobot(SimRobot::Object* robot);

  /**
   * Write the current game controller data to the object provided.
   * @param gameControllerData The object the game controller data is written to.
   */
  void getGameControllerData(GameControllerData& gameControllerData);

  /**
   * Write the current whistle to the object provided.
   * @param whistle The object the whistle is written to.
   */
  void getWhistle(Whistle& whistle);

private:
  /**
   * Finds a free place for a (un)penalized robot.
   * @param robot The number of the robot to place [0 ... numOfRobots-1].
   * @param x The optimal x coordinate. Might be moved toward own goal.
   * @param y The y coordinate.
   * @param rotation The rotation when placed.
   */
  void placeForPenalty(int robot, float x, float y, float rotation);

  /**
   * Checks if the position of a robot is illegal at the transition from ready to set.
   * @param robot The number of the robot to check the position of.
   */
  bool checkIllegalPositionInSet(int robot) const;

  /** Adds the time that has elapsed in the current state to timeBeforeCurrentState. */
  void addTimeInCurrentState();

  /** Sets all times when penalized to 0. */
  void resetPenaltyTimes();

  /** Resets the sets of players that touched the ball. */
  void resetBallContacts();

  /** Update the ball position based on the rules. */
  BallOut updateBall();

  /**
   * Initialize both teams.
   * @param robotsPlaying The maximum number of players per team playing at the same time.
   * @param messageBudget The message budget per team for a whole game.
   * @param goalkeepers The player numbers of the goalkeepers of both teams.
   */
  void initTeams(const uint8_t robotsPlaying, const uint16_t messageBudget, const std::array<uint8_t, 2>& goalkeepers = {1, 1});

  /**
   * Places or hides the referee next to the field showing a signal.
   * @param signal The signal that will be shown.
   */
  void showReferee(const RefereeSignal signal) const;

  friend class TestGameController;
};
