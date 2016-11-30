/**
 * @file Controller/GameController.h
 * This file implements a class that simulates a console-based GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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
Pose2f GameController::lastBallContactPose;

const float GameController::footLength = 120.f;
const float GameController::safeDistance = 150.f;
const float GameController::dropHeight = 350.f;

GameController::GameController()
{
  gameInfo.gameType = GAME_ROUNDROBIN;
  gameInfo.playersPerTeam = numOfRobots / 2;
  gameInfo.firstHalf = 1;
  gameInfo.kickOffTeam = 1;
  gameInfo.dropInTime = -1;
  gameInfo.secsRemaining = durationOfHalf;
  teamInfos[TEAM_BLUE].teamNumber = 1;
  teamInfos[TEAM_BLUE].teamColor = TEAM_BLUE;
  teamInfos[TEAM_RED].teamNumber = 2;
  teamInfos[TEAM_RED].teamColor = TEAM_RED;
}

void GameController::registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot)
{
  ASSERT(!robots[robot].simulatedRobot);
  robots[robot].simulatedRobot = &simulatedRobot;
  robots[robot].info.number = robot % (numOfRobots / 2) + 1;
  if(fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}

bool GameController::handleGlobalCommand(const std::string& command)
{
  if(command == "initial")
  {
    gameInfo.state = STATE_INITIAL;
    timeOfLastDropIn = timeWhenHalfStarted = 0;
    gameInfo.secsRemaining = durationOfHalf;
    return true;
  }
  else if(command == "ready")
  {
    gameInfo.state = STATE_READY;
    for(int i = 0; i < numOfRobots; ++i)
      if(robots[i].info.penalty)
        handleRobotCommand(i, "none");
    timeWhenStateBegan = Time::getCurrentSystemTime();
    return true;
  }
  else if(command == "set")
  {
    gameInfo.state = STATE_SET;
    for(int i = 0; i < numOfRobots; ++i)
      robots[i].info.penalty = none;

    if(automatic)
    {
      placeGoalie(0);
      placeGoalie(numOfRobots / 2);
      placeDefensivePlayers(gameInfo.kickOffTeam == 1 ? numOfRobots / 2 + 1 : 1);
      placeOffensivePlayers(gameInfo.kickOffTeam == 1 ? 1 : numOfRobots / 2 + 1);
      executePlacement();
    }

    timeWhenStateBegan = Time::getCurrentSystemTime();
    SimulatedRobot::moveBall(Vector3f(0.f, 0.f, 50.f), true);
    return true;
  }
  else if(command == "playing")
  {
    gameInfo.state = STATE_PLAYING;
    if(gameInfo.gameType == GAME_PLAYOFF || !timeWhenHalfStarted)
      timeWhenHalfStarted = Time::getCurrentSystemTime() - (durationOfHalf - gameInfo.secsRemaining) * 1000;
    return true;
  }
  else if(command == "finished")
  {
    gameInfo.state = STATE_FINISHED;
    return true;
  }
  else if(command == "kickOffBlue")
  {
    gameInfo.kickOffTeam = 1;
    return true;
  }
  else if(command == "kickOffRed")
  {
    gameInfo.kickOffTeam = 2;
    return true;
  }
  else if(command == "outByBlue")
  {
    gameInfo.dropInTeam = 1;
    timeOfLastDropIn = Time::getCurrentSystemTime();
    return true;
  }
  else if(command == "outByRed")
  {
    gameInfo.dropInTeam = 2;
    timeOfLastDropIn = Time::getCurrentSystemTime();
    return true;
  }
  else if(command == "gamePlayoff")
  {
    gameInfo.gameType = GAME_PLAYOFF;
    return true;
  }
  else if(command == "gameRoundRobin")
  {
    gameInfo.gameType = GAME_ROUNDROBIN;
    return true;
  }
  else if(command == "gameDropIn")
  {
    gameInfo.gameType = GAME_DROPIN;
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
    if(command == getName(i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (uint8_t)i;
      tr.penalty = r.info.penalty;
      tr.secsTillUnpenalised = 45;
      if(i)
      {
        r.timeWhenPenalized = Time::getCurrentSystemTime();
        if(automatic)
          placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark,
                          fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
      }
      else if(automatic)
      {
        ASSERT(r.simulatedRobot);
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0 ? fieldDimensions.yPosRightSideline - safeDistance : fieldDimensions.yPosLeftSideline + safeDistance,
                        ballPos.y() >= 0 ? pi_2 : -pi_2);
      }
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
  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];
    if(r.info.penalty)
    {
      r.info.secsTillUnpenalised = (uint8_t)(std::max(int(45 - Time::getTimeSince(r.timeWhenPenalized) / 1000), 0));
      RoboCup::RobotInfo& tr = teamInfos[i * 2 / numOfRobots].players[i % (numOfRobots / 2)];
      tr.secsTillUnpenalised = r.info.secsTillUnpenalised;

      if(automatic && r.info.secsTillUnpenalised <= 0)
      {
        r.info.penalty = PENALTY_NONE;
        tr.penalty = PENALTY_NONE;

        ASSERT(r.simulatedRobot);
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0 ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y() >= 0 ? pi_2 : -pi_2);
      }
    }
  }

  if(automatic)
  {
    SYNC;
    switch(gameInfo.state)
    {
      case STATE_READY:
        if(Time::getTimeSince(timeWhenStateBegan) < 2000)
          timeWhenLastRobotMoved = 0;
        if(Time::getTimeSince(timeWhenStateBegan) >= 45000 ||
           (timeWhenLastRobotMoved && Time::getTimeSince(timeWhenLastRobotMoved) > 2000))
          handleGlobalCommand("set");
        break;

      case STATE_SET:
        if(Time::getTimeSince(timeWhenStateBegan) >= 5000)
          handleGlobalCommand("playing");
        break;

      case STATE_PLAYING:
        switch(updateBall())
        {
          case goalByBlue:
            ++teamInfos[TEAM_BLUE].score;
            VERIFY(handleGlobalCommand("kickOffRed"));
            VERIFY(handleGlobalCommand("ready"));
            break;
          case goalByRed:
            ++teamInfos[TEAM_RED].score;
            VERIFY(handleGlobalCommand("kickOffBlue"));
            VERIFY(handleGlobalCommand("ready"));
            break;
          case outByBlue:
            VERIFY(handleGlobalCommand("outByBlue"));
            break;
          case outByRed:
            VERIFY(handleGlobalCommand("outByRed"));
            break;
          case notOut:
            break;
        }
    }
  }
}

GameController::BallOut GameController::updateBall()
{
  BallOut result = notOut;
  Vector2f ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  Vector2f ballInnerEdge(ballPos.x() - sgn(ballPos.x()) * (fieldDimensions.ballRadius + fieldDimensions.fieldLinesWidth / 2.f),
                         ballPos.y() - sgn(ballPos.y()) * (fieldDimensions.ballRadius + fieldDimensions.fieldLinesWidth / 2.f));
  if(!fieldDimensions.isInsideField(ballInnerEdge))
  {
    if(std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal) // goal
      result = ballPos.x() > 0 ? goalByRed : goalByBlue;
    else
    {
      float x;
      if((Pose2f(ballPos) - lastBallContactPose).translation.x() > 0) // 1m behind robot
        x = (lastBallContactPose + Pose2f(-1000, 0)).translation.x();
      else // 1m behind where ball went out
        x = (Pose2f(lastBallContactPose.rotation, ballPos) + Pose2f(-1000, 0)).translation.x();

      if(std::abs(ballPos.x()) > fieldDimensions.xPosOpponentGroundline && (Pose2f(x, 0) - Pose2f(lastBallContactPose.rotation)).translation.x() > 0)
        x = 0; // center line
      else if(x < fieldDimensions.xPosOwnDropInLine)
        x = fieldDimensions.xPosOwnDropInLine; // clip
      else if(x > fieldDimensions.xPosOpponentDropInLine)
        x = fieldDimensions.xPosOpponentDropInLine; // clip
      ballPos.x() = x;

      if(ballPos.y() < 0)
        ballPos.y() = fieldDimensions.yPosRightDropInLine; // right throw-in line
      else
        ballPos.y() = fieldDimensions.yPosLeftDropInLine; // left throw-in line

      SimulatedRobot::moveBall(Vector3f(ballPos.x(), ballPos.y(), 100.f), true);
      result = lastBallContactPose.rotation == 0 ? outByRed : outByBlue;
    }
  }
  return result;
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2f(SimulatedRobot::isBlue(robot) ? pi : 0, SimulatedRobot::getPosition(robot));
}

void GameController::writeGameInfo(Out& stream)
{
  SYNC;
  if(timeOfLastDropIn)
    gameInfo.dropInTime = (unsigned short)(Time::getTimeSince(timeOfLastDropIn) / 1000);
  else
    gameInfo.dropInTime = -1;
  if(gameInfo.state == STATE_PLAYING || (gameInfo.gameType != GAME_PLAYOFF && timeWhenHalfStarted))
    gameInfo.secsRemaining = (uint16_t)(durationOfHalf - Time::getTimeSince(timeWhenHalfStarted) / 1000);
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
    "kickOffBlue",
    "kickOffRed",
    "outByBlue",
    "outByRed",
    "gameDropIn"
    "gamePlayoff",
    "gameRoundRobin"
  };
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  FOREACH_ENUM(Penalty, i)
    completion.insert(std::string("pr ") + getName(i));
}
