/**
 * @file SimulatedNao/GameController.cpp
 * This file implements a class that simulates a console-based GameController.
 * @author Thomas Röfer
 * @author Arne Hasselbring
 */

#include "Tools/Communication/TeamMessageChannel.h"
#include "GameController.h"
#include "SimulatedRobot.h"
#include "Framework/Settings.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Representations/Configuration/RobotDimensions.h"
#include <algorithm>
#include <SimRobotCore3.h>

GameController::GameController() :
  theTeamMessageChannel(new TeamMessageChannel[2] { {inTeamMessage, outTeamMessage}, {inTeamMessage, outTeamMessage} })
{
  gameControllerData.packetNumber = 0;
  gameControllerData.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
  gameControllerData.competitionType = COMPETITION_TYPE_NORMAL;
  gameControllerData.gamePhase = GAME_PHASE_NORMAL;
  gameControllerData.state = STATE_INITIAL;
  gameControllerData.setPlay = SET_PLAY_NONE;
  gameControllerData.firstHalf = 1;
  gameControllerData.kickingTeam = 0; // is initialized in setTeamInfos.
  gameControllerData.secsRemaining = halfTime;
  gameControllerData.secondaryTime = 0;
  competitionTypeChampionsCup();
  for(std::size_t i = 0; i < 2; ++i)
    for(std::size_t j = 0; j < MAX_NUM_PLAYERS; ++j)
      robots[i * MAX_NUM_PLAYERS + j].info = &gameControllerData.teams[i].players[j];
  // Force reloading of the field dimensions (they cannot be loaded here because the file search path is not available yet).
  fieldDimensions.xPosOwnPenaltyMark = 0.f;
  resetBallContacts();
}

GameController::~GameController()
{
  delete[] theTeamMessageChannel;
}

void GameController::setTeamInfos(const std::array<TeamInfo, 2>& teamInfos)
{
  gameControllerData.teams[0].teamNumber = teamInfos[0].number;
  gameControllerData.teams[0].fieldPlayerColor = teamInfos[0].fieldPlayerColor;
  gameControllerData.teams[0].goalkeeperColor = teamInfos[0].goalkeeperColor;
  gameControllerData.teams[1].teamNumber = teamInfos[1].number;
  gameControllerData.teams[1].fieldPlayerColor = teamInfos[1].fieldPlayerColor;
  gameControllerData.teams[1].goalkeeperColor = teamInfos[1].goalkeeperColor;
  gameControllerData.kickingTeam = teamInfos[0].number;
  theTeamMessageChannel[0].startLocal(Settings::getPortForTeam(teamInfos[0].number), 12);
  theTeamMessageChannel[1].startLocal(Settings::getPortForTeam(teamInfos[1].number), 12);
}

void GameController::loadBallSpecification()
{
  InMapFile stream("ballSpecification.cfg");
  if(stream.exists())
    stream >> ballSpecification;
  RobotDimensions robotDimensions;
  InMapFile stream2("robotDimensions.cfg");
  if(stream2.exists())
  {
    stream2 >> robotDimensions;
    footLength = robotDimensions.footLength + 10.f;
    dropHeight = robotDimensions.simOriginHeight + 20.f;
    penaltyPlacementDistance = robotDimensions.armOffset.y() * 2.f + 300.f;
  }
}

void GameController::registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot)
{
  static_assert(numOfRobots == SimulatedRobot::robotsPerTeam * 2);
  ASSERT(!robots[robot].simulatedRobot);
  robots[robot].simulatedRobot = &simulatedRobot;
  if(robot % MAX_NUM_PLAYERS < gameControllerData.playersPerTeam)
    robots[robot].info->penalty = PENALTY_NONE;
  robots[robot].lastPenalty = robots[robot].info->penalty;
  if(fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}

bool GameController::initial()
{
  if(gameControllerData.state == STATE_INITIAL)
    return true;

  resetPenaltyTimes();
  timeBeforeCurrentState = 0;
  timeWhenStateBegan = Time::getCurrentSystemTime();
  gameControllerData.state = STATE_INITIAL;
  gameControllerData.setPlay = SET_PLAY_NONE;
  kickOffReason = kickOffReasonHalf;
  return true;
}

bool GameController::standby()
{
  if(gameControllerData.gamePhase != GAME_PHASE_NORMAL)
    return false;
  if(gameControllerData.state == STATE_STANDBY)
    return true;

  timeWhenStateBegan = Time::getCurrentSystemTime();
  gameControllerData.state = STATE_STANDBY;
  gameControllerData.setPlay = SET_PLAY_NONE;
  kickOffReason = kickOffReasonHalf;
  return true;
}

bool GameController::ready()
{
  if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT)
    return false;
  if(gameControllerData.state == STATE_READY)
    return true;

  if(gameControllerData.state == STATE_INITIAL)
    resetPenaltyTimes();
  else if(gameControllerData.state == STATE_PLAYING)
    addTimeInCurrentState();
  timeWhenStateBegan = Time::getCurrentSystemTime();
  gameControllerData.state = STATE_READY;
  if(gameControllerData.setPlay != SET_PLAY_PENALTY_KICK)
    gameControllerData.setPlay = SET_PLAY_NONE;
  return true;
}

bool GameController::set()
{
  if(gameControllerData.state == STATE_SET)
    return true;

  if(gameControllerData.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
    addTimeInCurrentState();
  timeWhenStateBegan = Time::getCurrentSystemTime();
  if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    timeBeforeCurrentState = 0;
    if(gameControllerData.state != STATE_INITIAL)
      gameControllerData.kickingTeam = gameControllerData.teams[gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? 1 : 0].teamNumber;

    for(int i = 0; i < numOfRobots; ++i)
      penalty(i, substitute);
  }
  gameControllerData.state = STATE_SET;
  if(gameControllerData.setPlay != SET_PLAY_PENALTY_KICK)
    gameControllerData.setPlay = SET_PLAY_NONE;

  return true;
}

bool GameController::playing()
{
  if(gameControllerData.state == STATE_PLAYING)
  {
    if(gameControllerData.setPlay != SET_PLAY_NONE)
    {
      showReferee(noSignal);
      gameControllerData.setPlay = SET_PLAY_NONE;
    }
    return true;
  }

  if(gameControllerData.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
    addTimeInCurrentState();

  gameControllerData.state = STATE_PLAYING;
  resetBallContacts();

  whistle.confidenceOfLastWhistleDetection = 2.f;
  whistle.channelsUsedForWhistleDetection = 2;
  whistle.lastTimeWhistleDetected = Time::getCurrentSystemTime();
  return true;
}

bool GameController::finished()
{
  if(gameControllerData.state == STATE_FINISHED)
    return true;

  resetPenaltyTimes();
  addTimeInCurrentState();
  timeWhenStateBegan = Time::getCurrentSystemTime();
  gameControllerData.state = STATE_FINISHED;
  gameControllerData.setPlay = SET_PLAY_NONE;
  return true;
}

bool GameController::competitionPhasePlayoff()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.competitionPhase = COMPETITION_PHASE_PLAYOFF;
  return true;
}

bool GameController::competitionPhaseRoundrobin()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
  return true;
}

bool GameController::competitionTypeChampionsCup()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.competitionType = COMPETITION_TYPE_NORMAL;
  initTeams(7, 1200);
  return true;
}

bool GameController::competitionTypeChallengeShield()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.competitionType = COMPETITION_TYPE_NORMAL;
  initTeams(5, 1200);
  return true;
}

bool GameController::globalGameStuck()
{
  if(gameControllerData.gamePhase != GAME_PHASE_NORMAL)
    return false;
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE))
    return false;
  gameControllerData.kickingTeam = KICKING_TEAM_NONE;
  kickOffReason = kickOffReasonGlobalGameStuck;
  VERIFY(ready());
  return true;
}

bool GameController::goal(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE))
    return false;
  if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT && gameControllerData.kickingTeam != gameControllerData.teams[side].teamNumber)
    return false;
  ++gameControllerData.teams[side].score;

  if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    kickOffReason = kickOffReasonPenalty;
    VERIFY(finished());
  }
  else
  {
    kickingTeamBeforeGoal = gameControllerData.kickingTeam;
    gameControllerData.kickingTeam = gameControllerData.teams[1 - side].teamNumber;
    kickOffReason = kickOffReasonGoal;
    VERIFY(ready());

    whistle.confidenceOfLastWhistleDetection = 2.f;
    whistle.channelsUsedForWhistleDetection = 4;
    whistle.lastTimeWhistleDetected = Time::getCurrentSystemTime();
  }
  return true;
}

bool GameController::goalKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_GOAL_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  showReferee(side == 0 ? right : left);
  resetBallContacts();
  return true;
}

bool GameController::pushingFreeKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_PUSHING_FREE_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  showReferee(side == 0 ? right : left);
  resetBallContacts();
  return true;
}

bool GameController::cornerKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_CORNER_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  showReferee(side == 0 ? right : left);
  resetBallContacts();
  return true;
}

bool GameController::kickIn(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_KICK_IN;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  showReferee(side == 0 ? right : left);
  resetBallContacts();
  return true;
}

bool GameController::teamPenaltyKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_PENALTY_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  showReferee(side == 0 ? right : left);
  kickOffReason = kickOffReasonPenalty;
  resetBallContacts();
  return ready();
}

bool GameController::kickOff(int side)
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  return true;
}

bool GameController::dropBall()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.kickingTeam = KICKING_TEAM_NONE;
  return true;
}

bool GameController::setHalf(int half)
{
  if(gameControllerData.state != STATE_INITIAL && gameControllerData.state != STATE_FINISHED)
    return false;
  gameControllerData.firstHalf = half == 1 ? 1 : 0;
  kickOffReason = kickOffReasonHalf;
  return true;
}

bool GameController::gamePhasePenaltyshoot()
{
  if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT)
    return true;
  gameControllerData.gamePhase = GAME_PHASE_PENALTYSHOOT;
  gameControllerData.state = STATE_INITIAL;
  gameControllerData.setPlay = SET_PLAY_NONE;
  gameControllerData.kickingTeam = gameControllerData.teams[0].teamNumber;
  timeBeforeCurrentState = 0;
  kickOffReason = kickOffReasonPenalty;
  return true;
}

bool GameController::gamePhaseNormal()
{
  if(gameControllerData.gamePhase == GAME_PHASE_NORMAL)
    return true;

  gameControllerData.gamePhase = GAME_PHASE_NORMAL;
  gameControllerData.state = STATE_INITIAL;
  gameControllerData.setPlay = SET_PLAY_NONE;
  timeBeforeCurrentState = 0;

  return true;
}

bool GameController::penalty(int robot, Penalty penalty)
{
  Robot& r = robots[robot];
  r.info->penalty = penalty == manual ? PENALTY_MANUAL
                    : penalty == substitute ? PENALTY_SUBSTITUTE
                    : penalty == foul || penalty == penaltyKick ? PENALTY_SPL_PLAYER_PUSHING
                    : static_cast<uint8_t>(penalty);
  if(penalty != none)
    r.timeWhenPenalized = Time::getCurrentSystemTime();
  if(penalty == foul)
    pushingFreeKick(1 - robot * 2 / numOfRobots);
  else if(penalty == penaltyKick)
    teamPenaltyKick(1 - robot * 2 / numOfRobots);
  return true;
}

void GameController::placeForPenalty(int robot, float x, float y, float rotation)
{
  Robot& r = robots[robot];
  ASSERT(r.simulatedRobot);
  Vector2f newPos(robot < numOfRobots / 2 ? x : -x, y);
  for(;;)
  {
    int j = 0;
    while(j < numOfRobots && (j == robot || !robots[j].simulatedRobot || (robots[j].lastPose.translation - newPos).squaredNorm() >= sqr(300.f)))
      ++j;
    if(j == numOfRobots)
    {
      r.lastPose = Pose2f(rotation, newPos);
      r.simulatedRobot->moveRobot(Vector3f(newPos.x(), newPos.y(), dropHeight), Vector3f(0.f, 0.f, rotation), true);
      break;
    }
    else
      newPos.x() += newPos.x() < 0.f ? -penaltyPlacementDistance : penaltyPlacementDistance;
  }
}

bool GameController::checkIllegalPositionInSet(int robot) const
{
  const Robot& r = robots[robot];
  if(!r.simulatedRobot || r.info->penalty != PENALTY_NONE)
    return false;

  const bool isFirstTeam = robot < numOfRobots / 2;
  const bool isKickingTeam = gameControllerData.kickingTeam == gameControllerData.teams[isFirstTeam ? 0 : 1].teamNumber;
  const bool isGoalkeeper = robot == 0 || robot == numOfRobots / 2;
  const bool isPenaltyKick = gameControllerData.setPlay == SET_PLAY_PENALTY_KICK;
  const bool notOnField = r.lastPose.translation.y() < fieldDimensions.yPosRightTouchline - footLength ||
                          r.lastPose.translation.y() > fieldDimensions.yPosLeftTouchline + footLength ||
                          r.lastPose.translation.x() < fieldDimensions.xPosOwnGoalLine - footLength ||
                          r.lastPose.translation.x() > fieldDimensions.xPosOpponentGoalLine + footLength;
  const bool inOpponentHalf = isFirstTeam ? (r.lastPose.translation.x() < footLength) : (r.lastPose.translation.x() > -footLength);
  const bool inCenterCircle = r.lastPose.translation.squaredNorm() < sqr(fieldDimensions.centerCircleRadius + footLength);
  const bool notOnOwnGoalLine = std::abs(r.lastPose.translation.x() - (isFirstTeam ? fieldDimensions.xPosOpponentGoalLine : fieldDimensions.xPosOwnGoalLine)) > footLength ||
                                r.lastPose.translation.y() > fieldDimensions.yPosLeftGoal || r.lastPose.translation.y() < fieldDimensions.yPosRightGoal;
  const bool inOpponentPenaltyArea = r.lastPose.translation.y() > fieldDimensions.yPosRightPenaltyArea - footLength && r.lastPose.translation.y() < fieldDimensions.yPosLeftPenaltyArea + footLength &&
                                     (isFirstTeam ? (r.lastPose.translation.x() < fieldDimensions.xPosOwnPenaltyArea + footLength) : (r.lastPose.translation.x() > fieldDimensions.xPosOpponentPenaltyArea - footLength));
  const bool inOwnPenaltyArea = r.lastPose.translation.y() > fieldDimensions.yPosRightPenaltyArea - footLength && r.lastPose.translation.y() < fieldDimensions.yPosLeftPenaltyArea + footLength &&
                                (isFirstTeam ? (r.lastPose.translation.x() > fieldDimensions.xPosOpponentPenaltyArea - footLength) : (r.lastPose.translation.x() < fieldDimensions.xPosOwnPenaltyArea + footLength));
  const Vector2f opponentPenaltyMark(isFirstTeam ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f);
  const bool onOpponentPenaltyMark = (r.lastPose.translation - opponentPenaltyMark).squaredNorm() < sqr(footLength + fieldDimensions.penaltyMarkSize * 0.5f);
  auto closestToOpponentPenaltyMark = [&]
  {
    const float myDistance2 = (r.lastPose.translation - opponentPenaltyMark).squaredNorm();
    const int base = isFirstTeam ? 0 : numOfRobots / 2;
    for(int i = base; i < base + numOfRobots / 2; ++i)
    {
      if(i == robot || !robots[i].simulatedRobot || robots[i].info->penalty != PENALTY_NONE)
        continue;
      const float itsDistance2 = (robots[i].lastPose.translation - opponentPenaltyMark).squaredNorm();
      if(itsDistance2 >= sqr(footLength + fieldDimensions.penaltyMarkSize * 0.5f) && (itsDistance2 < myDistance2 || (itsDistance2 == myDistance2 && i < robot)))
        return false;
    }
    return true;
  };

  return (notOnField ||
          (isPenaltyKick ?
           (isKickingTeam ? onOpponentPenaltyMark || (inOpponentPenaltyArea && !closestToOpponentPenaltyMark()) : (isGoalkeeper ? notOnOwnGoalLine : inOwnPenaltyArea)) :
           (isKickingTeam ? (inOpponentHalf && !inCenterCircle) : (inOpponentHalf || inCenterCircle))));
}

void GameController::update()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];
    if(r.simulatedRobot)
    {
      Pose2f pose;
      r.simulatedRobot->getRobotPose(pose);
      if(i < numOfRobots / 2)
        pose = Pose2f(pi) + pose;
      if((pose.translation - r.lastPose.translation).squaredNorm() > sqr(5.f) ||
         std::abs(Angle::normalize(pose.rotation - r.lastPose.rotation)) > 0.05f)
      {
        timeWhenLastRobotMoved = Time::getCurrentSystemTime();
        r.lastPose = pose;
      }
    }
  }

  if(lastState != STATE_SET && gameControllerData.state == STATE_SET)
  {
    if(automatic & bit(penalizeIllegalPositionInSet) && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT)
    {
      for(int i = 0; i < numOfRobots; ++i)
      {
        if(checkIllegalPositionInSet(i))
        {
          Robot& r = robots[i];
          r.info->penalty = PENALTY_SPL_ILLEGAL_POSITION_IN_SET;
          r.timeWhenPenalized = Time::getCurrentSystemTime();
        }
      }
    }

    if(automatic & bit(placeBall))
    {
      if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT || gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
        SimulatedRobot::moveBall(Vector3f(gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f, ballSpecification.radius), true);
      else
        SimulatedRobot::moveBall(Vector3f(0.f, 0.f, ballSpecification.radius), true);
    }
  }
  lastState = gameControllerData.state;

  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];

    // Prepare numbers for illegal positioning.
    if(automatic & bit(penalizeIllegalPosition))
    {
      if(r.info->penalty == PENALTY_NONE && r.simulatedRobot)
      {
        const bool isFirstTeam = i < numOfRobots / 2;
        const float yMarginGoalArea = std::min(r.lastPose.translation.y() - fieldDimensions.yPosRightGoalArea + footLength, fieldDimensions.yPosLeftGoalArea + footLength - r.lastPose.translation.y());
        const float yMarginPenaltyArea = std::min(r.lastPose.translation.y() - fieldDimensions.yPosRightPenaltyArea + footLength, fieldDimensions.yPosLeftPenaltyArea + footLength - r.lastPose.translation.y());
        const float xMarginOwnGoalArea = isFirstTeam ? (r.lastPose.translation.x() - fieldDimensions.xPosOpponentGoalArea + footLength) : (fieldDimensions.xPosOwnGoalArea + footLength - r.lastPose.translation.x());
        const float xMarginOwnPenaltyArea = isFirstTeam ? (r.lastPose.translation.x() - fieldDimensions.xPosOpponentPenaltyArea + footLength) : (fieldDimensions.xPosOwnPenaltyArea + footLength - r.lastPose.translation.x());
        const float xMarginOpponentPenaltyArea = isFirstTeam ? (fieldDimensions.xPosOwnPenaltyArea + footLength - r.lastPose.translation.x()) : (r.lastPose.translation.x() - fieldDimensions.xPosOpponentPenaltyArea + footLength);
        r.ownGoalAreaMargin = std::min(yMarginGoalArea, xMarginOwnGoalArea);
        r.ownPenaltyAreaMargin = std::min(yMarginPenaltyArea, xMarginOwnPenaltyArea);
        r.opponentPenaltyAreaMargin = std::min(yMarginPenaltyArea, xMarginOpponentPenaltyArea);
      }
      else
        r.ownGoalAreaMargin = r.ownPenaltyAreaMargin = r.opponentPenaltyAreaMargin = -1.f;
    }
  }

  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];

    if(r.info->penalty == PENALTY_NONE && r.simulatedRobot)
    {
      auto inOpponentHalfBeforeBallIsInPlay = [&]
      {
        if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT ||
           gameControllerData.state != STATE_PLAYING || gameControllerData.setPlay != SET_PLAY_NONE ||
           lastBallContactTime >= timeWhenStateBegan || Time::getTimeSince(timeWhenStateBegan) >= 10000 ||
           gameControllerData.kickingTeam == KICKING_TEAM_NONE)
          return false;
        const bool isFirstTeam = i < numOfRobots / 2;
        const bool isKickingTeam = gameControllerData.kickingTeam == gameControllerData.teams[isFirstTeam ? 0 : 1].teamNumber;
        const bool inOpponentHalf = isFirstTeam ? (r.lastPose.translation.x() < footLength) : (r.lastPose.translation.x() > -footLength);
        const bool inCenterCircle = r.lastPose.translation.squaredNorm() < sqr(fieldDimensions.centerCircleRadius + footLength);
        return isKickingTeam ? (inOpponentHalf && !inCenterCircle) : (inOpponentHalf || inCenterCircle);
      };
      auto inPenaltyAreaDuringPenaltyKick = [&]
      {
        if(gameControllerData.state != STATE_PLAYING || gameControllerData.setPlay != SET_PLAY_PENALTY_KICK)
          return false;
        const bool isFirstTeam = i < numOfRobots / 2;
        const bool isKickingTeam = gameControllerData.kickingTeam == gameControllerData.teams[isFirstTeam ? 0 : 1].teamNumber;
        // Outside the penalty area is legal.
        if((isKickingTeam ? r.opponentPenaltyAreaMargin : r.ownPenaltyAreaMargin) < 0.f)
          return false;
        // The goalkeeper is only allowed to stand on the goal line.
        // (but if it's outside the penalty area, that's okay, too, because that will happen when it's unpenalized during a penalty kick).
        if(!isKickingTeam && i == (isFirstTeam ? 0 : numOfRobots / 2) &&
           std::abs(r.lastPose.translation.x() - (isFirstTeam ? fieldDimensions.xPosOpponentGoalLine : fieldDimensions.xPosOwnGoalLine)) < footLength &&
           r.lastPose.translation.y() < fieldDimensions.yPosLeftGoal && r.lastPose.translation.y() > fieldDimensions.yPosRightGoal)
          return false;
        // No other defender may be in there.
        if(!isKickingTeam)
          return true;
        // The player of the kicking team which is deepest inside the opponent's penalty area is considered to be the taker which is allowed to be there.
        // All others are not.
        for(int j = isFirstTeam ? 0 : numOfRobots / 2; j < (isFirstTeam ? numOfRobots / 2 : numOfRobots); ++j)
          if(i != j && (robots[j].opponentPenaltyAreaMargin > r.opponentPenaltyAreaMargin || (robots[j].opponentPenaltyAreaMargin == r.opponentPenaltyAreaMargin && j < i)))
            return true;
        return false;
      };
      auto inFullOwnGoalArea = [&]
      {
        if(gameControllerData.state != STATE_PLAYING)
          return false;
        const bool isFirstTeam = i < numOfRobots / 2;
        // Count the number of players which are deeper inside the own goal area.
        // If more than 3 players are, this player must be illegal.
        if(r.ownGoalAreaMargin >= 0.f)
        {
          int counter = 0;
          for(int j = isFirstTeam ? 0 : numOfRobots / 2; j < (isFirstTeam ? numOfRobots / 2 : numOfRobots); ++j)
            if(i != j && (robots[j].ownGoalAreaMargin > r.ownGoalAreaMargin || (robots[j].ownGoalAreaMargin == r.ownGoalAreaMargin && j < i)))
              ++counter;
          if(counter >= 3)
            return true;
        }
        return false;
      };
      auto inFreeKickArea = [&]
      {
        const bool isFirstTeam = i < numOfRobots / 2;
        if(gameControllerData.state != STATE_PLAYING || gameControllerData.setPlay == SET_PLAY_NONE || gameControllerData.setPlay == SET_PLAY_PENALTY_KICK ||
           gameControllerData.kickingTeam == gameControllerData.teams[isFirstTeam ? 0 : 1].teamNumber)
          return false;

        // For now, just penalize anyone who is within 75cm after 10s.
        // TODO: This does not consider:
        //   - Players that clearly violate the rule by walking to or playing the ball before 10s
        //   - Players that are inside the circle after 10s because they are fallen or pushed
        Vector2f ballPos;
        return SimulatedRobot::getAbsoluteBallPosition(ballPos) && Time::getTimeSince(timeWhenSetPlayBegan) > 10000 && (r.lastPose.translation - ballPos).squaredNorm() < sqr(750.f);
      };

      if(automatic & bit(penalizeLeavingTheField) && !fieldDimensions.isInsideCarpet(r.lastPose.translation))
        VERIFY(penalty(i, leavingTheField));
      else if(automatic & bit(penalizeIllegalPosition) &&
              (inOpponentHalfBeforeBallIsInPlay() || inPenaltyAreaDuringPenaltyKick() || inFullOwnGoalArea() || inFreeKickArea()))
        VERIFY(penalty(i, illegalPosition));
    }

    if(automatic & bit(placePlayers) && r.info->penalty != PENALTY_NONE && r.lastPenalty == PENALTY_NONE && r.simulatedRobot)
    {
      placeForPenalty(i, -fieldDimensions.xPosReturnFromPenalty,
                      r.lastPose.translation.y() > 0.f ? fieldDimensions.yPosLeftReturnFromPenalty : fieldDimensions.yPosRightReturnFromPenalty,
                      r.lastPose.translation.y() > 0.f ? -pi_2 : pi_2);
    }

    if(r.info->penalty != PENALTY_NONE)
    {
      r.info->secsTillUnpenalised = static_cast<uint8_t>(std::max<int>((r.info->penalty == PENALTY_SPL_ILLEGAL_POSITION_IN_SET ? 15 : 45) - Time::getTimeSince(r.timeWhenPenalized) / 1000, 0));

      if(automatic & bit(unpenalize) && r.info->secsTillUnpenalised <= 0 && r.info->penalty != PENALTY_SPL_REQUEST_FOR_PICKUP && r.info->penalty != PENALTY_MANUAL && r.info->penalty != PENALTY_SUBSTITUTE)
        r.info->penalty = PENALTY_NONE;
    }
    else
      r.info->secsTillUnpenalised = 0;

    if(automatic & bit(placePlayers) && r.info->penalty == PENALTY_NONE && r.lastPenalty != PENALTY_NONE && r.simulatedRobot)
    {
      if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT)
      {
        Pose2f newPose((i < numOfRobots / 2) ? pi : 0.f, 0.f, 0.f);
        if(gameControllerData.kickingTeam == gameControllerData.teams[i < numOfRobots / 2 ? 0 : 1].teamNumber)
          newPose.translate(fieldDimensions.xPosPenaltyStrikerStartPosition, 0.f);
        else
          newPose.translate(fieldDimensions.xPosOwnGoalLine, 0.f);
        r.simulatedRobot->moveRobot(Vector3f(newPose.translation.x(), newPose.translation.y(), dropHeight), Vector3f(0.f, 0.f, newPose.rotation), true);
      }
    }

    if(automatic & bit(clearBall) && r.simulatedRobot)
    {
      Vector2f ballPos;
      if(r.simulatedRobot->getAbsoluteBallPosition(ballPos))
      {
        if((r.lastPose * Vector2f(50.f, 0.f) - ballPos).norm() > ballSpecification.radius)
          r.timeWhenBallNotStuckBetweenLegs = Time::getCurrentSystemTime();
        else if(r.timeWhenBallNotStuckBetweenLegs && Time::getTimeSince(r.timeWhenBallNotStuckBetweenLegs) > 500)
          SimulatedRobot::moveBall((Vector3f() << r.lastPose * Vector2f(ballSpecification.radius + 100.f, 0.f), ballSpecification.radius).finished(), true);
      }
    }

    r.lastPenalty = r.info->penalty;
  }

  switch(gameControllerData.state)
  {
    case STATE_STANDBY:
      if(Time::getTimeSince(timeWhenStateBegan) > (refereeStandbyDelay + refereeStandbyTime) * 1000)
        showReferee(noSignal);
      else if(Time::getTimeSince(timeWhenStateBegan) > refereeStandbyDelay * 1000)
        showReferee(up);
      [[fallthrough]];

    case STATE_READY:
      if(Time::getTimeSince(timeWhenStateBegan) < 2000)
        timeWhenLastRobotMoved = 0;
      if(Time::getTimeSince(timeWhenStateBegan) >= (gameControllerData.setPlay == SET_PLAY_PENALTY_KICK ? penaltyKickReadyTime : readyTime) * 1000 ||
         (automatic & bit(switchToSet) && timeWhenLastRobotMoved && Time::getTimeSince(timeWhenLastRobotMoved) > 2000))
        set();
      break;

    case STATE_SET:
      if(automatic & bit(switchToPlaying) && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT && Time::getTimeSince(timeWhenStateBegan) >= 5000)
        playing();
      break;

    case STATE_PLAYING:
      const bool wasPenaltyKick = gameControllerData.setPlay == SET_PLAY_PENALTY_KICK;
      if(gameControllerData.setPlay != SET_PLAY_NONE)
      {
        if(Time::getTimeSince(timeWhenSetPlayBegan) >= refereeSetPlayTime * 1000)
          showReferee(noSignal);
        if(gameControllerData.setPlay != SET_PLAY_PENALTY_KICK
           ? Time::getTimeSince(timeWhenSetPlayBegan) >= freeKickTime * 1000
           : Time::getTimeSince(timeWhenStateBegan) >= penaltyShotTime * 1000)
        {
          gameControllerData.setPlay = SET_PLAY_NONE;
          ballContacts[gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? 1 : 0] = 1;
        }
      }

      if(automatic & bit(freeKickComplete) && gameControllerData.setPlay != SET_PLAY_NONE &&
         lastBallContactTime > (gameControllerData.setPlay != SET_PLAY_PENALTY_KICK ? timeWhenSetPlayBegan + 500 : timeWhenStateBegan) && (gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber) != (lastBallContactPose.rotation == 0.f))
        VERIFY(playing());

      if(wasPenaltyKick)
        ballContacts[gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? 0 : 1] = 1;

      if(automatic & bit(ballOut))
      {
        const auto ballOutType = updateBall();
        if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT && ballOutType != (gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? goalByFirstTeam : goalBySecondTeam) && ballOutType != notOut)
          VERIFY(finished());
        else if(ballOutType != notOut)
        {
          // It is possible that setPlay is not NONE here because the auto-referee may
          // not have detected a completed free kick or an opponent (which actually is
          // not allowed to do so) touched the ball.
          gameControllerData.setPlay = SET_PLAY_NONE;
          switch(ballOutType)
          {
            case goalByFirstTeam:
              VERIFY(goal(0));
              break;
            case goalBySecondTeam:
              VERIFY(goal(1));
              break;
            case kickInFirstTeam:
              VERIFY(kickIn(0));
              break;
            case kickInSecondTeam:
              VERIFY(kickIn(1));
              break;
            case cornerKickFirstTeam:
              VERIFY(cornerKick(0));
              break;
            case cornerKickSecondTeam:
              VERIFY(cornerKick(1));
              break;
            case goalKickFirstTeam:
              VERIFY(goalKick(0));
              break;
            case goalKickSecondTeam:
              VERIFY(goalKick(1));
              break;
            default:
              break;
          }
        }
      }
  }

  auto getRemainingGameTime = [this](bool real)
  {
    const int duration = gameControllerData.gamePhase == GAME_PHASE_NORMAL ? halfTime : penaltyShotTime;
    const int timePlayed = gameControllerData.state == STATE_INITIAL
                           || gameControllerData.state == STATE_STANDBY
                           || ((gameControllerData.state == STATE_READY || gameControllerData.state == STATE_SET)
                               && ((gameControllerData.competitionPhase == COMPETITION_PHASE_PLAYOFF &&
                                    (real || gameControllerData.gamePhase != GAME_PHASE_NORMAL || gameControllerData.state != STATE_READY || kickOffReason != kickOffReasonGoal || Time::getTimeSince(timeWhenStateBegan) >= delayedSwitchAfterGoal * 1000)) || timeBeforeCurrentState == 0))
                           || gameControllerData.state == STATE_FINISHED
                           ? timeBeforeCurrentState / 1000
                           : real || (gameControllerData.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState > 0) || gameControllerData.state != STATE_PLAYING || Time::getTimeSince(timeWhenStateBegan) >= delayedSwitchToPlaying * 1000
                           ? Time::getTimeSince(timeWhenStateBegan - timeBeforeCurrentState) / 1000
                           : timeBeforeCurrentState / 1000;
    return duration - timePlayed;
  };
  gameControllerData.secsRemaining = static_cast<int16_t>(getRemainingGameTime(automatic & bit(trueGameState)));

  if(automatic & bit(switchToFinished) && getRemainingGameTime(true) <= 0)
    finished();

  if(!(automatic & bit(trueGameState)) && ((gameControllerData.state == STATE_PLAYING && Time::getTimeSince(timeWhenStateBegan) < delayedSwitchToPlaying * 1000) ||
                                           (gameControllerData.state == STATE_READY && kickOffReason == kickOffReasonGoal && Time::getTimeSince(timeWhenStateBegan) < delayedSwitchAfterGoal * 1000)))
    gameControllerData.secondaryTime = 0;
  else if(gameControllerData.state == STATE_READY)
    gameControllerData.secondaryTime = static_cast<int16_t>((gameControllerData.setPlay == SET_PLAY_PENALTY_KICK ? penaltyKickReadyTime : readyTime) - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else if(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
    gameControllerData.secondaryTime = static_cast<int16_t>(penaltyShotTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else if(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay != SET_PLAY_NONE)
    gameControllerData.secondaryTime = static_cast<int16_t>(freeKickTime - Time::getTimeSince(timeWhenSetPlayBegan) / 1000);
  else if((automatic & bit(kickOffDelay)) && gameControllerData.state == STATE_PLAYING && kickOffReason != kickOffReasonPenalty && kickOffTime >= Time::getTimeSince(timeWhenStateBegan) / 1000)
    gameControllerData.secondaryTime = static_cast<int16_t>(kickOffTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else
    gameControllerData.secondaryTime = 0;

  ++gameControllerData.packetNumber;

  for(int team = 0; team < 2; ++team)
  {
    while(theTeamMessageChannel[team].receive())
    {
      if(gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT
         && gameControllerData.state != STATE_INITIAL
         && gameControllerData.state != STATE_FINISHED
         && gameControllerData.teams[team].messageBudget > 0)
        --gameControllerData.teams[team].messageBudget;
    }
  }

  gameControllerData.timeLastPacketReceived = Time::getCurrentSystemTime();
  gameControllerData.isTrueData = automatic & bit(trueGameState);
}

void GameController::addTimeInCurrentState()
{
  timeBeforeCurrentState += Time::getCurrentSystemTime() - timeWhenStateBegan;
}

void GameController::resetPenaltyTimes()
{
  for(auto& robot : robots)
    robot.timeWhenPenalized = 0;
}

void GameController::resetBallContacts()
{
  for(auto& ballContact : ballContacts)
    ballContact = 0;
  lastBallContactRobots[0] = lastBallContactRobots[1] = nullptr;
}

GameController::BallOut GameController::updateBall()
{
  BallOut result = notOut;
  Vector2f ballPos;
  if(!SimulatedRobot::getAbsoluteBallPosition(ballPos))
    return result;

  const Vector2f ballInnerEdge(ballPos.x() - sgn(ballPos.x()) * (ballSpecification.radius + fieldDimensions.fieldLinesWidth / 2.f),
                               ballPos.y() - sgn(ballPos.y()) * (ballSpecification.radius + fieldDimensions.fieldLinesWidth / 2.f));
  if(!fieldDimensions.isInsideField(ballInnerEdge))
  {
    // Goals can be scored in penalty shootouts, without having touched the ball (opponent own goals),
    // or with at least two robots having touched the ball.
    const int ballContact = ballContacts[static_cast<int>(ballPos.x() > 0.f)];
    const bool canScore = gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT
                          || automatic & bit(directGoals) || !ballContact || (ballContact & (ballContact - 1)) != 0;
    const bool insideGoal = std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal;
    if(insideGoal && canScore) // goal
      result = ballPos.x() > 0.f ? goalBySecondTeam : goalByFirstTeam;
    else
    {
      if(std::abs(ballPos.x()) > fieldDimensions.xPosOpponentGoalLine)
      {
        if((ballPos.x() > 0.f) == (lastBallContactPose.rotation == 0.f) || insideGoal) // goal kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGoalArea : fieldDimensions.xPosOwnGoalArea;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightGoalArea;
          else
            ballPos.y() = fieldDimensions.yPosLeftGoalArea;

          result = ballPos.x() > 0.f ? goalKickFirstTeam : goalKickSecondTeam;
        }
        else // corner kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGoalLine : fieldDimensions.xPosOwnGoalLine;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightTouchline;
          else
            ballPos.y() = fieldDimensions.yPosLeftTouchline;

          result = ballPos.x() > 0.f ? cornerKickSecondTeam : cornerKickFirstTeam;
        }
      }
      else // kick in
      {
        float x = ballPos.x();
        if(x < fieldDimensions.xPosOwnGoalLine)
          x = fieldDimensions.xPosOwnGoalLine; // clip
        else if(x > fieldDimensions.xPosOpponentGoalLine)
          x = fieldDimensions.xPosOpponentGoalLine; // clip
        ballPos.x() = x;

        if(ballPos.y() < 0.f)
          ballPos.y() = fieldDimensions.yPosRightTouchline;
        else
          ballPos.y() = fieldDimensions.yPosLeftTouchline;

        result = lastBallContactPose.rotation == 0.f ? kickInFirstTeam : kickInSecondTeam;
      }

      SimulatedRobot::moveBall(Vector3f(ballPos.x(), ballPos.y(), 100.f), true);
    }
  }
  return result;
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  const size_t teamIndex = 1 - static_cast<size_t>(SimulatedRobot::isFirstTeam(robot));
  int& ballContact = ballContacts[teamIndex];
  if((ballContact & 1) && ballContact > 1 && Time::getTimeSince(lastBallContactTime) > 100)
    ballContact &= ~1;
  ballContact |= 1 << SimulatedRobot::getNumber(robot);
  lastBallContactPose = Pose2f(teamIndex ? 0.f : pi, SimulatedRobot::getPosition(robot));
  lastBallContactTime = Time::getCurrentSystemTime();
  lastBallContactRobots[teamIndex] = robot;
}

void GameController::getGameControllerData(GameControllerData& gameControllerData)
{
  gameControllerData = this->gameControllerData;
  if(!(automatic & bit(trueGameState)))
  {
    if(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE &&
       Time::getTimeSince(timeWhenStateBegan) < delayedSwitchToPlaying * 1000)
      gameControllerData.state = STATE_SET;
    else if(gameControllerData.gamePhase == GAME_PHASE_NORMAL &&
            gameControllerData.state == STATE_READY &&
            kickOffReason == kickOffReasonGoal &&
            Time::getTimeSince(timeWhenStateBegan) < delayedSwitchAfterGoal * 1000)
    {
      gameControllerData.state = STATE_PLAYING;
      --gameControllerData.teams[gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? 1 : 0].score;
      gameControllerData.kickingTeam = kickingTeamBeforeGoal;
    }
    if(gameControllerData.gamePhase == GAME_PHASE_NORMAL &&
       (gameControllerData.state == STATE_PLAYING || gameControllerData.setPlay != SET_PLAY_NONE))
      gameControllerData.kickingTeam = KICKING_TEAM_NONE;
  }
}

void GameController::getWhistle(Whistle& whistle)
{
  whistle = this->whistle;
}

void GameController::initTeams(const uint8_t robotsPlaying, const uint16_t messageBudget, const std::array<uint8_t, 2>& goalkeepers)
{
  gameControllerData.playersPerTeam = robotsPlaying;
  for(auto& teamInfo : gameControllerData.teams)
  {
    const ptrdiff_t side = &teamInfo - gameControllerData.teams;
    teamInfo.goalkeeper = goalkeepers[side];
    teamInfo.messageBudget = messageBudget;
    for(int j = 0; j < MAX_NUM_PLAYERS; ++j)
      teamInfo.players[j].penalty = j >= robotsPlaying ? PENALTY_SUBSTITUTE
                                    : robots[j + MAX_NUM_PLAYERS * side].simulatedRobot ? PENALTY_NONE : PENALTY_SPL_REQUEST_FOR_PICKUP;
  }
}

void GameController::showReferee(const RefereeSignal signal) const
{
  if(!refereeObject)
    return;
  Vector3f translation = {0.f, -3.350f, 0.025f};
  Vector3f angles = {0_deg, 0_deg, -90_deg};
  switch(signal)
  {
    case noSignal:
      translation.y() = -4000.f;
      break;
    case up:
      angles.z() = 90_deg;
      break;
    case left:
      angles.x() = 180_deg;
      angles.z() = 90_deg;
      break;
    case right:
      angles.x() = 180_deg;
  }
  RotationMatrix rotation = RotationMatrix::fromEulerAngles(angles);
  float robotRotationConverted[3][3];
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      robotRotationConverted[i][j] = rotation(i, j);
  static_cast<SimRobotCore3::Body*>(refereeObject)->move(translation.data(), robotRotationConverted);
  static_cast<SimRobotCore3::Body*>(refereeObject)->resetDynamics();
}
