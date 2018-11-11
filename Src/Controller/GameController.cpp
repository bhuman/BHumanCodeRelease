/**
 * @file Controller/GameController.h
 * This file implements a class that simulates a console-based GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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

FieldDimensions GameController::fieldDimensions;
BallSpecification GameController::ballSpecification;
Pose2f GameController::lastBallContactPose;

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
  gameInfo.dropInTeam = 0;
  gameInfo.dropInTime = -1;
  gameInfo.secsRemaining = halfTime;
  gameInfo.secondaryTime = 0;
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
    timeOfLastDropIn = timeBeforeCurrentState = 0;
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
  else if(command == "competitionTypeGeneralPenaltyKick")
  {
    gameInfo.competitionType = COMPETITION_TYPE_GENERAL_PENALTY_KICK;
    VERIFY(handleGlobalCommand("gamePenaltyShootout"));
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

bool GameController::handleOutCommand(const std::string& command)
{
  if(!(gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_NONE && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;
  else if(command == "outByFirstTeam")
  {
    timeOfLastDropIn = Time::getCurrentSystemTime();
    gameInfo.dropInTeam = 1;
    return true;
  }
  else if(command == "outBySecondTeam")
  {
    timeOfLastDropIn = Time::getCurrentSystemTime();
    gameInfo.dropInTeam = 2;
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
  else if(handleOutCommand(command))
    return true;
  else if(handleFreeKickCommand(command))
    return true;
  else if(handleKickOffCommand(command))
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
    while(j < numOfRobots && (j == robot || !robots[j].simulatedRobot || (robots[j].lastPose.translation - newPos).norm() >= 300))
      ++j;
    if(j == numOfRobots)
    {
      r.lastPose = Pose2f(rotation, newPos);
      r.simulatedRobot->moveRobot(Vector3f(newPos.x(), newPos.y(), dropHeight), Vector3f(0, 0, rotation), true);
      break;
    }
    else
      newPos.x() += newPos.x() < 0 ? -400 : 400;
  }
}

bool GameController::inOwnPenaltyArea(int robot) const
{
  const Robot& r = robots[robot];
  if(r.lastPose.translation.y() < fieldDimensions.yPosRightPenaltyArea ||
     r.lastPose.translation.y() > fieldDimensions.yPosLeftPenaltyArea)
    return false;
  else if(robot < numOfRobots / 2)
    return r.lastPose.translation.x() >= fieldDimensions.xPosOpponentPenaltyArea &&
           (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGroundline ||
            (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGoal &&
             r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
  else
    return r.lastPose.translation.x() <= fieldDimensions.xPosOwnPenaltyArea &&
           (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGroundline ||
            (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGoal &&
             r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
}

void GameController::moveMe(int robot, Vector3f position, Vector3f rotation)
{
  robots[robot - 1].simulatedRobot->moveRobot(position, rotation, true);
}

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  r.manuallyPlaced = r.simulatedRobot &&
                     (r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
                      r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
                      (robot < numOfRobots / 2 && (r.lastPose.translation.x() < footLength ||
                          r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
                      (robot >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength ||
                          r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
  if(r.manuallyPlaced)
    r.lastPose = robot < numOfRobots / 2 ? Pose2f(-pi, fieldDimensions.xPosOpponentGroundline - safeDistance, 0)
                 : Pose2f(0, fieldDimensions.xPosOwnGroundline + safeDistance, 0);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2f* poses)
{
  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(i != robot && robots[i].simulatedRobot)
    {
      const Robot& r2 = robots[i];
      float minDistance = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for(int j = 0; j < numOfFieldPlayers; ++j)
      {
        const Pose2f& pose = poses[j];
        float distance = (pose.translation - r2.lastPose.translation).norm();
        if(!occupied[j] && distance < minDistance)
        {
          minDistance = distance;
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
      Pose2f(0, -fieldDimensions.centerCircleRadius - footLength, 0),
      Pose2f(0, fieldDimensions.xPosOwnPenaltyMark, fieldDimensions.yPosRightGoal),
      Pose2f(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftPenaltyArea),
      Pose2f(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightPenaltyArea)
    },
    {
      Pose2f(-pi, fieldDimensions.centerCircleRadius + footLength, 0),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosLeftGoal),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftPenaltyArea),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightPenaltyArea)
    }
  };

  // Move all field players that are not in their own half.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.simulatedRobot &&
                       (r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 && (r.lastPose.translation.x() < footLength ||
                            r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength ||
                            r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }

  freePenaltyArea(minRobot, poses[minRobot < numOfRobots / 2 ? 1 : 0]);
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
    r.manuallyPlaced = r.simulatedRobot &&
                       (r.lastPose.translation.norm() < fieldDimensions.centerCircleRadius + footLength ||
                        r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 && (r.lastPose.translation.x() < footLength ||
                            r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength ||
                            r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }

  freePenaltyArea(minRobot, poses[minRobot < numOfRobots / 2 ? 1 : 0]);
}

void GameController::freePenaltyArea(int minRobot, const Pose2f* poses)
{
  // Count robots in penalty area and determine the one that is
  // furthest away from the field center.
  int numOfRobotsInOwnPenaltyArea = 0;
  float maxDistance = -1.f;
  int indexOfMaxDistance = 0;
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(inOwnPenaltyArea(i))
    {
      ++numOfRobotsInOwnPenaltyArea;
      float distance = robots[i].lastPose.translation.squaredNorm();
      if(distance > maxDistance)
      {
        maxDistance = distance;
        indexOfMaxDistance = i;
      }
    }

  if(numOfRobotsInOwnPenaltyArea > 1)
  {
    // Move all remaining robots that are in the penalty area away
    for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
      if(inOwnPenaltyArea(i) && i != indexOfMaxDistance)
      {
        robots[i].manuallyPlaced = true;
        placeFromSet(i, minRobot, poses);
      }
  }
}

void GameController::executePlacement()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    const Robot& r = robots[i];
    if(r.manuallyPlaced)
      r.simulatedRobot->moveRobot(Vector3f(r.lastPose.translation.x(), r.lastPose.translation.y(), dropHeight),
                                  Vector3f(0, 0, r.lastPose.rotation), true);
  }
}

void GameController::referee()
{
  SYNC;

  if(automatic && lastState != STATE_SET && gameInfo.state == STATE_SET)
  {
    if(gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
    {
      placeGoalie(0);
      placeGoalie(numOfRobots / 2);
      placeDefensivePlayers(gameInfo.kickingTeam == 1 ? numOfRobots / 2 + 1 : 1);
      placeOffensivePlayers(gameInfo.kickingTeam == 1 ? 1 : numOfRobots / 2 + 1);
      executePlacement();
      SimulatedRobot::moveBall(Vector3f(0.f, 0.f, 50.f), true);
    }
    else
      SimulatedRobot::moveBall(Vector3f(gameInfo.kickingTeam == 1 ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f, 50.f), true);
  }
  lastState = gameInfo.state;

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
      r.info.secsTillUnpenalised = (uint8_t)(std::max(int(45 - Time::getTimeSince(r.timeWhenPenalized) / 1000), 0));
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
        Pose2f newPose((i < numOfRobots / 2) ? pi : 0, 0, 0);
        if(gameInfo.kickingTeam == (i < numOfRobots / 2 ? 1 : 2))
        {
          if(gameInfo.competitionType == COMPETITION_TYPE_GENERAL_PENALTY_KICK)
          {
            static const float angles[6] = { -60_deg, -30_deg, 0_deg, 30_deg, 60_deg, 0_deg };
            const int index = Random::uniformInt(0, 5);
            newPose.translate(fieldDimensions.xPosOpponentPenaltyMark, 0).rotate(angles[index]).translate(fieldDimensions.xPosPenaltyStrikerStartPosition - fieldDimensions.xPosOpponentPenaltyMark, 0);
          }
          else
            newPose.translate(fieldDimensions.xPosPenaltyStrikerStartPosition, 0);
        }
        else
          newPose.translate(fieldDimensions.xPosOwnGroundline, 0);
        r.simulatedRobot->moveRobot(Vector3f(newPose.translation.x(), newPose.translation.y(), dropHeight), Vector3f(0, 0, newPose.rotation), true);
      }
      else
      {
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0 ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y() >= 0 ? pi_2 : -pi_2);
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
        const auto ballOut = updateBall();
        if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && ballOut != (gameInfo.kickingTeam == 1 ? goalByFirstTeam : goalBySecondTeam) && ballOut != notOut)
        {
          VERIFY(handleGlobalCommand("finished"));
        }
        else
          switch(ballOut)
          {
            case goalByFirstTeam:
              VERIFY(handleGlobalCommand("goalByFirstTeam"));
              break;
            case goalBySecondTeam:
              VERIFY(handleGlobalCommand("goalBySecondTeam"));
              break;
            case outByFirstTeam:
              VERIFY(handleGlobalCommand("outByFirstTeam"));
              break;
            case outBySecondTeam:
              VERIFY(handleGlobalCommand("outBySecondTeam"));
              break;
            case goalOutByFirstTeam:
              VERIFY(handleGlobalCommand("goalFreeKickForSecondTeam"));
              break;
            case goalOutBySecondTeam:
              VERIFY(handleGlobalCommand("goalFreeKickForFirstTeam"));
              break;
            case notOut:
              break;
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
      result = ballPos.x() > 0 ? goalBySecondTeam : goalByFirstTeam;
    else
    {
      if(std::abs(ballPos.x()) > fieldDimensions.xPosOpponentGroundline && (ballPos.x() > 0) == (lastBallContactPose.rotation == 0)) // goal free kick
      {
        // "in line with the penalty spot and [the] side-edge of the penalty box" (section 3.7 of the 2018 rules)
        ballPos.x() = ballPos.x() > 0 ? fieldDimensions.xPosOpponentPenaltyMark : fieldDimensions.xPosOwnPenaltyMark;

        if(ballPos.y() < 0)
          ballPos.y() = fieldDimensions.yPosRightPenaltyArea;
        else
          ballPos.y() = fieldDimensions.yPosLeftPenaltyArea;

        result = lastBallContactPose.rotation == 0 ? goalOutBySecondTeam : goalOutByFirstTeam;
      }
      else
      {
        float x = (lastBallContactPose + Pose2f(-1000, 0)).translation.x(); // 1m behind robot
        if(x < fieldDimensions.xPosOwnDropInLine)
          x = fieldDimensions.xPosOwnDropInLine; // clip
        else if(x > fieldDimensions.xPosOpponentDropInLine)
          x = fieldDimensions.xPosOpponentDropInLine; // clip
        ballPos.x() = x;

        if(ballPos.y() < 0)
          ballPos.y() = fieldDimensions.yPosRightDropInLine; // right throw-in line
        else
          ballPos.y() = fieldDimensions.yPosLeftDropInLine; // left throw-in line

        result = lastBallContactPose.rotation == 0 ? outBySecondTeam : outByFirstTeam;
      }

      SimulatedRobot::moveBall(Vector3f(ballPos.x(), ballPos.y(), 100.f), true);
    }
  }
  return result;
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2f(SimulatedRobot::isFirstTeam(robot) ? pi : 0, SimulatedRobot::getPosition(robot));
}

void GameController::writeGameInfo(Out& stream)
{
  SYNC;

  if(timeOfLastDropIn)
    gameInfo.dropInTime = static_cast<uint16_t>(Time::getTimeSince(timeOfLastDropIn) / 1000);
  else
    gameInfo.dropInTime = -1;

  const int duration = gameInfo.gamePhase == GAME_PHASE_NORMAL ? halfTime : penaltyShotTime;
  const int timePlayed = gameInfo.state == STATE_INITIAL
    || ((gameInfo.state == STATE_READY || gameInfo.state == STATE_SET)
        && (gameInfo.competitionPhase == COMPETITION_PHASE_PLAYOFF || timeBeforeCurrentState == 0))
    || gameInfo.state == STATE_FINISHED
      ? timeBeforeCurrentState / 1000
      : Time::getTimeSince(timeWhenStateBegan - timeBeforeCurrentState) / 1000;
  gameInfo.secsRemaining = static_cast<uint16_t>(duration - timePlayed);

  if(gameInfo.state == STATE_READY)
    gameInfo.secondaryTime = static_cast<uint16_t>(readyTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && (gameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK || gameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK))
    gameInfo.secondaryTime = static_cast<uint16_t>(freeKickTime - Time::getTimeSince(timeWhenSetPlayBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && kickOffTime >= Time::getTimeSince(timeWhenStateBegan) / 1000)
    gameInfo.secondaryTime = static_cast<uint16_t>(kickOffTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else
    gameInfo.secondaryTime = 0;

  gameInfo.timeLastPackageReceived = Time::getCurrentSystemTime();
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
  if((pose.translation - r.lastPose.translation).norm() > 5 ||
     Angle::normalize(pose.rotation - r.lastPose.rotation) > 0.05)
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
    "competitionTypeGeneralPenaltyKick",
    "competitionPhasePlayoff",
    "competitionPhaseRoundRobin",
    "goalByFirstTeam",
    "goalBySecondTeam",
    "outByFirstTeam",
    "outBySecondTeam",
    "goalFreeKickForFirstTeam",
    "goalFreeKickForSecondTeam",
    "pushingFreeKickForFirstTeam",
    "pushingFreeKickForSecondTeam",
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

void GameController::setTeamInfos(Settings::TeamColor& firstTeamColor, Settings::TeamColor& secondTeamColor)
{
  teamInfos[0].teamNumber = 1;
  teamInfos[0].teamColor = firstTeamColor;
  teamInfos[1].teamNumber = 2;
  teamInfos[1].teamColor = secondTeamColor;
}
