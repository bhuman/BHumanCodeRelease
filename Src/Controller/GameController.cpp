/**
 * @file Controller/GameController.h
 * This file implements a class that simulates a console-based GameController.
 * @author Thomas Röfer
 * @author Arne Hasselbring
 */

#include "GameController.h"
#include "SimulatedRobot.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Eigen.h"
#include <limits>
#include <algorithm>

const float GameController::footLength = 120.f;
const float GameController::safeDistance = 150.f;
const float GameController::dropHeight = 350.f;

GameController::GameController()
{
  gameInfo.packetNumber = 0;
  gameInfo.playersPerTeam = numOfRobots / 2;
  gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
  gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
  gameInfo.gamePhase = GAME_PHASE_NORMAL;
  gameInfo.state = STATE_INITIAL;
  gameInfo.setPlay = SET_PLAY_NONE;
  gameInfo.firstHalf = 1;
  gameInfo.kickingTeam = 1;
  gameInfo.secsRemaining = halfTime;
  gameInfo.secondaryTime = 0;
  // Force reloading of the field dimensions (they cannot be loaded here because the file search path is not available yet).
  fieldDimensions.xPosOwnPenaltyMark = 0.f;
}

void GameController::registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot)
{
  ASSERT(!robots[robot].simulatedRobot);
  robots[robot].simulatedRobot = &simulatedRobot;
  robots[robot].info.number = robot % (numOfRobots / 2) + 1;
  if(fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}

bool GameController::handleStateCommand(const std::string& command)
{
  if(command == "initial")
  {
    if(gameInfo.state == STATE_INITIAL)
      return true;

    resetPenaltyTimes();
    timeBeforeCurrentState = 0;
    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_INITIAL;
    gameInfo.setPlay = SET_PLAY_NONE;
    return true;
  }
  else if(command == "ready")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      return false;
    if(gameInfo.state == STATE_READY)
      return true;

    resetPenaltyTimes();
    if(gameInfo.state == STATE_PLAYING)
      addTimeInCurrentState();
    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_READY;
    gameInfo.setPlay = SET_PLAY_NONE;

    return true;
  }
  else if(command == "set")
  {
    if(gameInfo.state == STATE_SET)
      return true;

    if(gameInfo.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
      addTimeInCurrentState();
    timeWhenStateBegan = Time::getCurrentSystemTime();
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    {
      timeBeforeCurrentState = 0;
      if(gameInfo.state != STATE_INITIAL)
        gameInfo.kickingTeam = gameInfo.kickingTeam == 1 ? 2 : 1;

      for(int i = 0; i < numOfRobots; ++i)
        VERIFY(handleRobotCommand(i, "substitute"));
    }
    gameInfo.state = STATE_SET;
    gameInfo.setPlay = SET_PLAY_NONE;

    return true;
  }
  else if(command == "playing")
  {
    if(gameInfo.state == STATE_PLAYING)
    {
      gameInfo.setPlay = SET_PLAY_NONE;
      return true;
    }

    if(gameInfo.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
      addTimeInCurrentState();

    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_PLAYING;
    return true;
  }
  else if(command == "finished")
  {
    if(gameInfo.state == STATE_FINISHED)
      return true;

    resetPenaltyTimes();
    addTimeInCurrentState();
    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_FINISHED;
    gameInfo.setPlay = SET_PLAY_NONE;
    return true;
  }
  return false;
}

bool GameController::handleCompetitionPhaseCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL)
    return false;
  else if(command == "competitionPhasePlayoff")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_PLAYOFF;
    return true;
  }
  else if(command == "competitionPhaseRoundRobin")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
    return true;
  }
  return false;
}

bool GameController::handleCompetitionTypeCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL)
    return false;
  else if(command == "competitionTypeNormal")
  {
    gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
    return true;
  }
  else if(command == "competitionTypeMixedTeam")
  {
    gameInfo.competitionType = COMPETITION_TYPE_MIXEDTEAM;
    return true;
  }
  return false;
}

bool GameController::handleGoalCommand(const std::string& command)
{
  if(!(gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_NONE))
    return false;
  else if(command == "goalByFirstTeam")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && gameInfo.kickingTeam != 1)
      return false;
    ++teamInfos[0].score;
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      VERIFY(handleStateCommand("finished"));
    else
    {
      gameInfo.kickingTeam = 2;
      VERIFY(handleStateCommand("ready"));
    }
    return true;
  }
  else if(command == "goalBySecondTeam")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && gameInfo.kickingTeam != 2)
      return false;
    ++teamInfos[1].score;
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      VERIFY(handleStateCommand("finished"));
    else
    {
      gameInfo.kickingTeam = 1;
      VERIFY(handleStateCommand("ready"));
    }
    return true;
  }
  return false;
}

bool GameController::handleFreeKickCommand(const std::string& command)
{
  if(!(gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_NONE && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  if(command == "goalFreeKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_GOAL_FREE_KICK;
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if(command == "goalFreeKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_GOAL_FREE_KICK;
    gameInfo.kickingTeam = 2;
    return true;
  }
  else if(command == "pushingFreeKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if(command == "pushingFreeKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    gameInfo.kickingTeam = 2;
    return true;
  }
  else if(command == "cornerKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if(command == "cornerKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    gameInfo.kickingTeam = 2;
    return true;
  }
  else if(command == "kickInForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if(command == "kickInForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    gameInfo.kickingTeam = 2;
    return true;
  }
  return false;
}

bool GameController::handleKickOffCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL)
    return false;
  else if(command == "kickOffFirstTeam")
  {
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if(command == "kickOffSecondTeam")
  {
    gameInfo.kickingTeam = 2;
    return true;
  }
  return false;
}

bool GameController::handleManualPlacementCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL && gameInfo.state != STATE_SET)
    return false;
  else if(command == "manualPlacementFirstTeam")
  {
    placeGoalie(0);
    if(gameInfo.kickingTeam == 1)
      placeOffensivePlayers(1);
    else
      placeDefensivePlayers(1);
    return true;
  }
  else if(command == "manualPlacementSecondTeam")
  {
    placeGoalie(numOfRobots / 2);
    if(gameInfo.kickingTeam == 2)
      placeOffensivePlayers(numOfRobots / 2 + 1);
    else
      placeDefensivePlayers(numOfRobots / 2 + 1);
    return true;
  }
  return false;
}

bool GameController::handleGlobalCommand(const std::string& command)
{
  if(handleStateCommand(command))
    return true;
  else if(handleCompetitionTypeCommand(command))
    return true;
  else if(handleCompetitionPhaseCommand(command))
    return true;
  else if(handleGoalCommand(command))
    return true;
  else if(handleFreeKickCommand(command))
    return true;
  else if(handleKickOffCommand(command))
    return true;
  else if(handleManualPlacementCommand(command))
    return true;
  else if(command == "gamePenaltyShootout")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      return true;

    gameInfo.gamePhase = GAME_PHASE_PENALTYSHOOT;
    gameInfo.state = STATE_INITIAL;
    gameInfo.setPlay = SET_PLAY_NONE;
    gameInfo.kickingTeam = 1;
    timeBeforeCurrentState = 0;

    return true;
  }
  else if(command == "gameNormal")
  {
    if(gameInfo.gamePhase == GAME_PHASE_NORMAL)
      return true;

    gameInfo.gamePhase = GAME_PHASE_NORMAL;
    gameInfo.state = STATE_INITIAL;
    gameInfo.setPlay = SET_PLAY_NONE;
    timeBeforeCurrentState = 0;

    return true;
  }
  return false;
}

bool GameController::handleGlobalConsole(In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleGlobalCommand(command);
}

bool GameController::handleRobotCommand(int robot, const std::string& command)
{
  Robot& r = robots[robot];
  RoboCup::RobotInfo& tr = teamInfos[robot * 2 / numOfRobots].players[robot % (numOfRobots / 2)];
  FOREACH_ENUM(Penalty, i)
    if(command == TypeRegistry::getEnumName(i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (i == substitute ? PENALTY_SUBSTITUTE : static_cast<uint8_t>(i));
      tr.penalty = r.info.penalty;
      if(i != none)
        r.timeWhenPenalized = Time::getCurrentSystemTime();
      return true;
    }
  return false;
}

bool GameController::handleRobotConsole(int robot, In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleRobotCommand(robot, command);
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

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  if(r.info.penalty != PENALTY_NONE)
    return;
  r.manuallyPlaced = true;
  r.lastPose = robot < numOfRobots / 2 ? Pose2f(-pi, fieldDimensions.xPosOpponentGroundline - safeDistance, 0.f)
               : Pose2f(0.f, fieldDimensions.xPosOwnGroundline + safeDistance, 0.f);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2f* poses)
{
  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(i != robot && robots[i].simulatedRobot && robots[i].info.penalty == PENALTY_NONE)
    {
      const Robot& r2 = robots[i];
      float minDistanceSqr = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for(int j = 0; j < numOfFieldPlayers; ++j)
      {
        const Pose2f& pose = poses[j];
        const float distanceSqr = (pose.translation - r2.lastPose.translation).squaredNorm();
        if(!occupied[j] && distanceSqr < minDistanceSqr)
        {
          minDistanceSqr = distanceSqr;
          bestPoseIndex = j;
        }
      }
      occupied[bestPoseIndex] = true;
    }

  // The position that would not be chosen is suitable for this robot.
  int i = 0;
  while(i < numOfFieldPlayers && occupied[i])
    ++i;
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] =
  {
    {
      Pose2f(0.f, -fieldDimensions.centerCircleRadius - footLength, 0.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)
    },
    {
      Pose2f(-pi, fieldDimensions.centerCircleRadius + footLength, 0.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)
    }
  };

  // Move all field players that are not in their own half.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    if(r.info.penalty != PENALTY_NONE)
      continue;
    r.manuallyPlaced = true;
    placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }
}

void GameController::placeDefensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] =
  {
    {
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)
    },
    {
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)
    }
  };

  // Move all field players that are not in their own half or in the center circle.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    if(r.info.penalty != PENALTY_NONE)
      continue;
    r.manuallyPlaced = true;
    placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }
}

void GameController::checkIllegalPositioning(int robot)
{
  Robot& r = robots[robot];
  if(!r.simulatedRobot || r.info.penalty != PENALTY_NONE)
    return;

  if(r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
     r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
     (robot < numOfRobots / 2 && (r.lastPose.translation.x() < footLength || r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
     (robot >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength || r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)) ||
     ((gameInfo.kickingTeam != (robot < numOfRobots / 2 ? 1 : 2)) && r.lastPose.translation.squaredNorm() < sqr(fieldDimensions.centerCircleRadius + footLength)))
  {
    RoboCup::RobotInfo& tr = teamInfos[robot * 2 / numOfRobots].players[robot % (numOfRobots / 2)];
    r.info.penalty = PENALTY_SPL_ILLEGAL_POSITIONING;
    tr.penalty = r.info.penalty;
    r.timeWhenPenalized = Time::getCurrentSystemTime();
  }
}

void GameController::executePlacement()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];
    if(r.manuallyPlaced)
      r.simulatedRobot->moveRobot(Vector3f(r.lastPose.translation.x(), r.lastPose.translation.y(), dropHeight),
                                  Vector3f(0.f, 0.f, r.lastPose.rotation), true);
    r.manuallyPlaced = false;
  }
}

void GameController::referee()
{
  SYNC;

  if(automatic && lastState != STATE_SET && gameInfo.state == STATE_SET)
  {
    if(gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
    {
      for(int i = 0; i < numOfRobots; ++i)
        checkIllegalPositioning(i);
      SimulatedRobot::moveBall(Vector3f(0.f, 0.f, 50.f), true);
    }
    else
      SimulatedRobot::moveBall(Vector3f(gameInfo.kickingTeam == 1 ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f, 50.f), true);
  }
  lastState = gameInfo.state;

  executePlacement();

  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];

    if(automatic && r.info.penalty != PENALTY_NONE && r.lastPenalty == PENALTY_NONE && r.simulatedRobot)
    {
      placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                      fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
    }

    if(r.info.penalty != PENALTY_NONE)
    {
      r.info.secsTillUnpenalised = static_cast<uint8_t>(std::max<int>((r.info.penalty == PENALTY_SPL_ILLEGAL_POSITIONING ? 15 : 45) - Time::getTimeSince(r.timeWhenPenalized) / 1000, 0));
      RoboCup::RobotInfo& tr = teamInfos[i * 2 / numOfRobots].players[i % (numOfRobots / 2)];
      tr.secsTillUnpenalised = r.info.secsTillUnpenalised;

      if(automatic && r.info.secsTillUnpenalised <= 0 && r.info.penalty != PENALTY_MANUAL && r.info.penalty != PENALTY_SUBSTITUTE)
      {
        r.info.penalty = PENALTY_NONE;
        tr.penalty = PENALTY_NONE;
      }
    }

    if(automatic && r.info.penalty == PENALTY_NONE && r.lastPenalty != PENALTY_NONE && r.simulatedRobot)
    {
      if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      {
        Pose2f newPose((i < numOfRobots / 2) ? pi : 0.f, 0.f, 0.f);
        if(gameInfo.kickingTeam == (i < numOfRobots / 2 ? 1 : 2))
          newPose.translate(fieldDimensions.xPosPenaltyStrikerStartPosition, 0.f);
        else
          newPose.translate(fieldDimensions.xPosOwnGroundline, 0.f);
        r.simulatedRobot->moveRobot(Vector3f(newPose.translation.x(), newPose.translation.y(), dropHeight), Vector3f(0.f, 0.f, newPose.rotation), true);
      }
      else
      {
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0.f ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y() >= 0.f ? pi_2 : -pi_2);
      }
    }

    r.lastPenalty = r.info.penalty;
  }

  switch(gameInfo.state)
  {
    case STATE_READY:
      if(Time::getTimeSince(timeWhenStateBegan) < 2000)
        timeWhenLastRobotMoved = 0;
      if(Time::getTimeSince(timeWhenStateBegan) >= readyTime * 1000 ||
         (automatic && timeWhenLastRobotMoved && Time::getTimeSince(timeWhenLastRobotMoved) > 2000))
        handleGlobalCommand("set");
      break;

    case STATE_SET:
      if(automatic && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && Time::getTimeSince(timeWhenStateBegan) >= 5000)
        handleGlobalCommand("playing");
      break;

    case STATE_PLAYING:
      if(Time::getTimeSince(timeWhenSetPlayBegan) >= freeKickTime * 1000)
        gameInfo.setPlay = SET_PLAY_NONE;

      if(automatic)
      {
        if(gameInfo.setPlay != SET_PLAY_NONE && lastBallContactTime > timeWhenSetPlayBegan + 500 && (gameInfo.kickingTeam == 1) != (lastBallContactPose.rotation == 0.f))
          VERIFY(handleGlobalCommand("playing"));

        const auto ballOut = updateBall();
        if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && ballOut != (gameInfo.kickingTeam == 1 ? goalByFirstTeam : goalBySecondTeam) && ballOut != notOut)
        {
          VERIFY(handleGlobalCommand("finished"));
        }
        else if(ballOut != notOut)
        {
          // It is possible that setPlay is not NONE here because the autoreferee may
          // not have detected a completed free kick or an opponent (which actually is
          // not allowed to do so) touched the ball.
          gameInfo.setPlay = SET_PLAY_NONE;
          switch(ballOut)
          {
            case goalByFirstTeam:
              VERIFY(handleGlobalCommand("goalByFirstTeam"));
              break;
            case goalBySecondTeam:
              VERIFY(handleGlobalCommand("goalBySecondTeam"));
              break;
            case outByFirstTeam:
              VERIFY(handleGlobalCommand("kickInForSecondTeam"));
              break;
            case outBySecondTeam:
              VERIFY(handleGlobalCommand("kickInForFirstTeam"));
              break;
            case ownGoalOutByFirstTeam:
              VERIFY(handleGlobalCommand("cornerKickForSecondTeam"));
              break;
            case ownGoalOutBySecondTeam:
              VERIFY(handleGlobalCommand("cornerKickForFirstTeam"));
              break;
            case opponentGoalOutByFirstTeam:
              VERIFY(handleGlobalCommand("goalFreeKickForSecondTeam"));
              break;
            case opponentGoalOutBySecondTeam:
              VERIFY(handleGlobalCommand("goalFreeKickForFirstTeam"));
              break;
            default:
              break;
          }
        }
      }
  }
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
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  Vector2f ballInnerEdge(ballPos.x() - sgn(ballPos.x()) * (ballSpecification.radius + fieldDimensions.fieldLinesWidth / 2.f),
                         ballPos.y() - sgn(ballPos.y()) * (ballSpecification.radius + fieldDimensions.fieldLinesWidth / 2.f));
  if(!fieldDimensions.isInsideField(ballInnerEdge))
  {
    if(std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal) // goal
      result = ballPos.x() > 0.f ? goalBySecondTeam : goalByFirstTeam;
    else
    {
      if(std::abs(ballPos.x()) > fieldDimensions.xPosOpponentGroundline)
      {
        if((ballPos.x() > 0.f) == (lastBallContactPose.rotation == 0.f)) // goal free kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentPenaltyMark : fieldDimensions.xPosOwnPenaltyMark;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightPenaltyArea;
          else
            ballPos.y() = fieldDimensions.yPosLeftPenaltyArea;

          result = lastBallContactPose.rotation == 0.f ? opponentGoalOutBySecondTeam : opponentGoalOutByFirstTeam;
        }
        else // corner kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGroundline : fieldDimensions.xPosOwnGroundline;

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
        if(x < fieldDimensions.xPosOwnGroundline)
          x = fieldDimensions.xPosOwnGroundline; // clip
        else if(x > fieldDimensions.xPosOpponentGroundline)
          x = fieldDimensions.xPosOpponentGroundline; // clip
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

void GameController::writeGameInfo(Out& stream)
{
  SYNC;

  const int duration = gameInfo.gamePhase == GAME_PHASE_NORMAL ? halfTime : penaltyShotTime;
  const int timePlayed = gameInfo.state == STATE_INITIAL
    || ((gameInfo.state == STATE_READY || gameInfo.state == STATE_SET)
        && (gameInfo.competitionPhase == COMPETITION_PHASE_PLAYOFF || timeBeforeCurrentState == 0))
    || gameInfo.state == STATE_FINISHED
      ? timeBeforeCurrentState / 1000
      : Time::getTimeSince(timeWhenStateBegan - timeBeforeCurrentState) / 1000;
  gameInfo.secsRemaining = static_cast<int16_t>(duration - timePlayed);

  if(gameInfo.state == STATE_READY)
    gameInfo.secondaryTime = static_cast<int16_t>(readyTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && gameInfo.setPlay != SET_PLAY_NONE)
    gameInfo.secondaryTime = static_cast<int16_t>(freeKickTime - Time::getTimeSince(timeWhenSetPlayBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && kickOffTime >= Time::getTimeSince(timeWhenStateBegan) / 1000)
    gameInfo.secondaryTime = static_cast<int16_t>(kickOffTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else
    gameInfo.secondaryTime = 0;

  gameInfo.timeLastPacketReceived = Time::getCurrentSystemTime();
  stream << gameInfo;
}

void GameController::writeOwnTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[robot * 2 / numOfRobots];
}

void GameController::writeOpponentTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[1 - robot * 2 / numOfRobots];
}

void GameController::writeRobotInfo(int robot, Out& stream)
{
  SYNC;
  Robot& r = robots[robot];
  Pose2f pose;
  ASSERT(r.simulatedRobot);
  r.simulatedRobot->getRobotPose(pose);
  if(robot < numOfRobots / 2)
    pose = Pose2f(pi) + pose;
  if((pose.translation - r.lastPose.translation).squaredNorm() > sqr(5.f) ||
     std::abs(Angle::normalize(pose.rotation - r.lastPose.rotation)) > 0.05f)
  {
    timeWhenLastRobotMoved = Time::getCurrentSystemTime();
    r.lastPose = pose;
  }
  stream << r.info;
}

void GameController::addCompletion(std::set<std::string>& completion) const
{
  static const char* commands[] =
  {
    "initial",
    "ready",
    "set",
    "playing",
    "finished",
    "competitionTypeNormal",
    "competitionTypeMixedTeam",
    "competitionPhasePlayoff",
    "competitionPhaseRoundRobin",
    "manualPlacementFirstTeam",
    "manualPlacementSecondTeam",
    "goalByFirstTeam",
    "goalBySecondTeam",
    "goalFreeKickForFirstTeam",
    "goalFreeKickForSecondTeam",
    "pushingFreeKickForFirstTeam",
    "pushingFreeKickForSecondTeam",
    "cornerKickForFirstTeam",
    "cornerKickForSecondTeam",
    "kickInForFirstTeam",
    "kickInForSecondTeam",
    "kickOffFirstTeam",
    "kickOffSecondTeam",
    "gamePenaltyShootout",
    "gameNormal"
  };
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  FOREACH_ENUM(Penalty, i)
    completion.insert(std::string("pr ") + TypeRegistry::getEnumName(i));
}

void GameController::setTeamInfos(Settings::TeamColor firstTeamColor, Settings::TeamColor secondTeamColor)
{
  teamInfos[0].teamNumber = 1;
  teamInfos[0].teamColor = firstTeamColor;
  teamInfos[1].teamNumber = 2;
  teamInfos[1].teamColor = secondTeamColor;
}
