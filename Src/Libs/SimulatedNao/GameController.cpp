/**
 * @file SimulatedNao/GameController.cpp
 * This file implements a class that simulates a console-based GameController.
 * @author Thomas Röfer
 * @author Arne Hasselbring
 */

#include "Tools/Communication/SPLMessageHandler.h"
#include "GameController.h"
#include "SimulatedRobot.h"
#include "Framework/Settings.h"
#include "Math/BHMath.h"
#include "Math/Eigen.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include <algorithm>
#include <limits>

const float GameController::footLength = 120.f;
const float GameController::dropHeight = 350.f;

GameController::GameController()
: theSPLMessageHandler(new SPLMessageHandler[2]{{inTeamMessages, outTeamMessage}, {inTeamMessages, outTeamMessage}})
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
  competitionTypeNormal();
  for(std::size_t i = 0; i < 2; ++i)
    for(std::size_t j = 0; j < MAX_NUM_PLAYERS; ++j)
      robots[i * MAX_NUM_PLAYERS + j].info = &gameControllerData.teams[i].players[j];
  // Force reloading of the field dimensions (they cannot be loaded here because the file search path is not available yet).
  fieldDimensions.xPosOwnPenaltyMark = 0.f;
}

GameController::~GameController()
{
  delete[] theSPLMessageHandler;
}

void GameController::setTeamInfos(const std::array<uint8_t, 2>& teamNumbers, const std::array<Settings::TeamColor, 2>& teamColors)
{
  gameControllerData.teams[0].teamNumber = teamNumbers[0];
  gameControllerData.teams[0].teamColor = teamColors[0];
  gameControllerData.teams[1].teamNumber = teamNumbers[1];
  gameControllerData.teams[1].teamColor = teamColors[1];
  gameControllerData.kickingTeam = teamNumbers[0];
  theSPLMessageHandler[0].startLocal(Settings::getPortForTeam(teamNumbers[0]), 12);
  theSPLMessageHandler[1].startLocal(Settings::getPortForTeam(teamNumbers[1]), 12);
}

void GameController::registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot)
{
  static_assert(numOfRobots == SimulatedRobot::robotsPerTeam * 2);
  ASSERT(!robots[robot].simulatedRobot);
  robots[robot].simulatedRobot = &simulatedRobot;
  if(robot % MAX_NUM_PLAYERS < robotsPlaying)
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
    gameControllerData.setPlay = SET_PLAY_NONE;
    return true;
  }

  if(gameControllerData.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
    addTimeInCurrentState();

  timeWhenStateBegan = Time::getCurrentSystemTime();
  gameControllerData.state = STATE_PLAYING;

  whistle.confidenceOfLastWhistleDetection = 2.f;
  whistle.channelsUsedForWhistleDetection = 4;
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

bool GameController::competitionTypeNormal()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.competitionType = COMPETITION_TYPE_NORMAL;
  initTeams(6, 5, 1200);
  return true;
}

bool GameController::competitionType7v7()
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.competitionType = COMPETITION_TYPE_7V7;
  initTeams(7, 7, 1680);
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
  return true;
}

bool GameController::pushingFreeKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_PUSHING_FREE_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  return true;
}

bool GameController::cornerKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_CORNER_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  return true;
}

bool GameController::kickIn(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_KICK_IN;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  return true;
}

bool GameController::penaltyKick(int side)
{
  if(!(gameControllerData.state == STATE_PLAYING && gameControllerData.setPlay == SET_PLAY_NONE && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  timeWhenSetPlayBegan = Time::getCurrentSystemTime();
  gameControllerData.setPlay = SET_PLAY_PENALTY_KICK;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
  kickOffReason = kickOffReasonPenalty;
  return ready();
}

bool GameController::kickOff(int side)
{
  if(gameControllerData.state != STATE_INITIAL)
    return false;
  gameControllerData.kickingTeam = gameControllerData.teams[side].teamNumber;
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
  r.info->penalty = penalty == manual ? PENALTY_MANUAL : (penalty == substitute ? PENALTY_SUBSTITUTE : static_cast<uint8_t>(penalty));
  if(penalty != none)
    r.timeWhenPenalized = Time::getCurrentSystemTime();
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
      newPos.x() += newPos.x() < 0.f ? -400.f : 400.f;
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
  const bool notOnField = r.lastPose.translation.y() < fieldDimensions.yPosRightSideline - footLength ||
                          r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline + footLength ||
                          r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundLine - footLength ||
                          r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundLine + footLength;
  const bool inOpponentHalf = isFirstTeam ? (r.lastPose.translation.x() < footLength) : (r.lastPose.translation.x() > -footLength);
  const bool inCenterCircle = r.lastPose.translation.squaredNorm() < sqr(fieldDimensions.centerCircleRadius + footLength);
  const bool notOnOwnGoalLine = std::abs(r.lastPose.translation.x() - (isFirstTeam ? fieldDimensions.xPosOpponentGroundLine : fieldDimensions.xPosOwnGroundLine)) > footLength ||
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
        SimulatedRobot::moveBall(Vector3f(gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f, 50.f), true);
      else
        SimulatedRobot::moveBall(Vector3f(0.f, 0.f, 50.f), true);
    }
  }
  lastState = gameControllerData.state;

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

    // Prepare numbers for illegal positioning.
    if(automatic & bit(penalizeIllegalPosition))
    {
      if(r.info->penalty == PENALTY_NONE && r.simulatedRobot)
      {
        const bool isFirstTeam = i < numOfRobots / 2;
        const float yMargin = std::min(r.lastPose.translation.y() - fieldDimensions.yPosRightPenaltyArea + footLength, fieldDimensions.yPosLeftPenaltyArea + footLength - r.lastPose.translation.y());
        const float xMarginOwn = isFirstTeam ? (r.lastPose.translation.x() - fieldDimensions.xPosOpponentPenaltyArea + footLength) : (fieldDimensions.xPosOwnPenaltyArea + footLength - r.lastPose.translation.x());
        const float xMarginOpponent = isFirstTeam ? (fieldDimensions.xPosOwnPenaltyArea + footLength - r.lastPose.translation.x()) : (r.lastPose.translation.x() - fieldDimensions.xPosOpponentPenaltyArea + footLength);
        r.ownPenaltyAreaMargin = std::min(yMargin, xMarginOwn);
        r.opponentPenaltyAreaMargin = std::min(yMargin, xMarginOpponent);
      }
      else
        r.ownPenaltyAreaMargin = r.opponentPenaltyAreaMargin = -1.f;
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
           lastBallContactTime >= timeWhenStateBegan || Time::getTimeSince(timeWhenStateBegan) >= 10000)
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
           std::abs(r.lastPose.translation.x() - (isFirstTeam ? fieldDimensions.xPosOpponentGroundLine : fieldDimensions.xPosOwnGroundLine)) < footLength &&
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
      auto inFullPenaltyArea = [&]
      {
        const bool isFirstTeam = i < numOfRobots / 2;
        // Count the number of players which are deeper inside the respective penalty area.
        // If more than 3 players are, this player must be illegal.
        if(r.ownPenaltyAreaMargin >= 0.f)
        {
          int counter = 0;
          for(int j = isFirstTeam ? 0 : numOfRobots / 2; j < (isFirstTeam ? numOfRobots / 2 : numOfRobots); ++j)
            if(i != j && (robots[j].ownPenaltyAreaMargin > r.ownPenaltyAreaMargin || (robots[j].ownPenaltyAreaMargin == r.ownPenaltyAreaMargin && j < i)))
              ++counter;
          if(counter >= 3)
            return true;
        }
        if(r.opponentPenaltyAreaMargin >= 0.f)
        {
          int counter = 0;
          for(int j = isFirstTeam ? 0 : numOfRobots / 2; j < (isFirstTeam ? numOfRobots / 2 : numOfRobots); ++j)
            if(i != j && (robots[j].opponentPenaltyAreaMargin > r.opponentPenaltyAreaMargin || (robots[j].opponentPenaltyAreaMargin == r.opponentPenaltyAreaMargin && j < i)))
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
              (inOpponentHalfBeforeBallIsInPlay() || inPenaltyAreaDuringPenaltyKick() || inFullPenaltyArea() || inFreeKickArea()))
        VERIFY(penalty(i, illegalPosition));
    }

    if(automatic & bit(placePlayers) && r.info->penalty != PENALTY_NONE && r.lastPenalty == PENALTY_NONE && r.simulatedRobot)
    {
      placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                      fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
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
          newPose.translate(fieldDimensions.xPosOwnGroundLine, 0.f);
        r.simulatedRobot->moveRobot(Vector3f(newPose.translation.x(), newPose.translation.y(), dropHeight), Vector3f(0.f, 0.f, newPose.rotation), true);
      }
      else
      {
        Vector2f ballPos = Vector2f::Zero();
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0.f ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y() >= 0.f ? pi_2 : -pi_2);
      }
    }

    if(automatic & bit(clearBall) && r.simulatedRobot)
    {
      Vector2f ballPos;
      if(r.simulatedRobot->getAbsoluteBallPosition(ballPos))
      {
        if((r.lastPose * Vector2f(50.f, 0.f) - ballPos).norm() > 50.f)
          r.timeWhenBallNotStuckBetweenLegs = Time::getCurrentSystemTime();
        else if(r.timeWhenBallNotStuckBetweenLegs && Time::getTimeSince(r.timeWhenBallNotStuckBetweenLegs) > 500)
          SimulatedRobot::moveBall((Vector3f() << r.lastPose * Vector2f(150.f, 0.f), 50.f).finished(), true);
      }
    }

    r.lastPenalty = r.info->penalty;
  }

  switch(gameControllerData.state)
  {
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
      if(gameControllerData.setPlay != SET_PLAY_NONE && (gameControllerData.setPlay != SET_PLAY_PENALTY_KICK ?
                                                         Time::getTimeSince(timeWhenSetPlayBegan) >= freeKickTime * 1000 :
                                                         Time::getTimeSince(timeWhenStateBegan) >= penaltyShotTime * 1000))
        gameControllerData.setPlay = SET_PLAY_NONE;

      if(automatic & bit(freeKickComplete) && gameControllerData.setPlay != SET_PLAY_NONE &&
         lastBallContactTime > (gameControllerData.setPlay != SET_PLAY_PENALTY_KICK ? timeWhenSetPlayBegan + 500 : timeWhenStateBegan) && (gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber) != (lastBallContactPose.rotation == 0.f))
        VERIFY(playing());

      if(automatic & bit(ballOut))
      {
        const auto ballOutType = updateBall();
        if(gameControllerData.gamePhase == GAME_PHASE_PENALTYSHOOT && ballOutType != (gameControllerData.kickingTeam == gameControllerData.teams[0].teamNumber ? goalByFirstTeam : goalBySecondTeam) && ballOutType != notOut)
          VERIFY(finished());
        else if(ballOutType != notOut)
        {
          // It is possible that setPlay is not NONE here because the autoreferee may
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
            case outByFirstTeam:
              VERIFY(kickIn(1));
              break;
            case outBySecondTeam:
              VERIFY(kickIn(0));
              break;
            case ownGoalOutByFirstTeam:
              VERIFY(cornerKick(1));
              break;
            case ownGoalOutBySecondTeam:
              VERIFY(cornerKick(0));
              break;
            case opponentGoalOutByFirstTeam:
              VERIFY(goalKick(1));
              break;
            case opponentGoalOutBySecondTeam:
              VERIFY(goalKick(0));
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
  else if(gameControllerData.state == STATE_PLAYING && gameControllerData.gamePhase != GAME_PHASE_PENALTYSHOOT && kickOffTime >= Time::getTimeSince(timeWhenStateBegan) / 1000)
    gameControllerData.secondaryTime = static_cast<int16_t>(kickOffTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else
    gameControllerData.secondaryTime = 0;

  ++gameControllerData.packetNumber;

  for(int team = 0; team < 2; ++team)
  {
    theSPLMessageHandler[team].receive();
    while(!inTeamMessages.empty())
    {
      inTeamMessages.removeFront();
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
    if(std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal) // goal
      result = ballPos.x() > 0.f ? goalBySecondTeam : goalByFirstTeam;
    else
    {
      if(std::abs(ballPos.x()) > fieldDimensions.xPosOpponentGroundLine)
      {
        if((ballPos.x() > 0.f) == (lastBallContactPose.rotation == 0.f)) // goal kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGoalArea : fieldDimensions.xPosOwnGoalArea;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightGoalArea;
          else
            ballPos.y() = fieldDimensions.yPosLeftGoalArea;

          result = lastBallContactPose.rotation == 0.f ? opponentGoalOutBySecondTeam : opponentGoalOutByFirstTeam;
        }
        else // corner kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGroundLine : fieldDimensions.xPosOwnGroundLine;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightSideline;
          else
            ballPos.y() = fieldDimensions.yPosLeftSideline;

          result = lastBallContactPose.rotation == 0.f ? ownGoalOutBySecondTeam : ownGoalOutByFirstTeam;
        }
      }
      else // kick in
      {
        float x = ballPos.x();
        if(x < fieldDimensions.xPosOwnGroundLine)
          x = fieldDimensions.xPosOwnGroundLine; // clip
        else if(x > fieldDimensions.xPosOpponentGroundLine)
          x = fieldDimensions.xPosOpponentGroundLine; // clip
        ballPos.x() = x;

        if(ballPos.y() < 0.f)
          ballPos.y() = fieldDimensions.yPosRightSideline;
        else
          ballPos.y() = fieldDimensions.yPosLeftSideline;

        result = lastBallContactPose.rotation == 0.f ? outBySecondTeam : outByFirstTeam;
      }

      SimulatedRobot::moveBall(Vector3f(ballPos.x(), ballPos.y(), 100.f), true);
    }
  }
  return result;
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2f(SimulatedRobot::isFirstTeam(robot) ? pi : 0.f, SimulatedRobot::getPosition(robot));
  lastBallContactTime = Time::getCurrentSystemTime();
}

void GameController::getGameControllerData(GameControllerData& gameControllerData)
{
  gameControllerData = this->gameControllerData;
  if(!(automatic & bit(trueGameState)))
  {
    if(gameControllerData.state == STATE_PLAYING &&
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
  }
}

void GameController::getWhistle(Whistle& whistle)
{
  whistle = this->whistle;
}

void GameController::initTeams(const uint8_t teamSize, const int robotsPlaying, const uint16_t messageBudget)
{
  this->robotsPlaying = robotsPlaying;
  gameControllerData.playersPerTeam = teamSize;
  for(auto& teamInfo : gameControllerData.teams)
  {
    teamInfo.messageBudget = messageBudget;
    for(int j = 0; j < MAX_NUM_PLAYERS; ++j)
      teamInfo.players[j].penalty = j >= robotsPlaying ? PENALTY_SUBSTITUTE
                                    : robots[j].simulatedRobot ? PENALTY_NONE : PENALTY_SPL_REQUEST_FOR_PICKUP;
  }
}
