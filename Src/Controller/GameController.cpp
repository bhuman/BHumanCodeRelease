/**
 * @file Controller/GameController.h
 * This file implements a class that simulates a console-based GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameController.h"
#include "SimulatedRobot.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include <limits>
#include <algorithm>

FieldDimensions GameController::fieldDimensions;
Pose2D GameController::lastBallContactPose;

// move to header when MSC supports constexpr
const float GameController::footLength = 120.f;
const float GameController::safeDistance = 150.f;
const float GameController::dropHeight = 350.f;

GameController::GameController() :
timeOfLastDropIn(0),
timeWhenLastRobotMoved(0),
timeWhenStateBegan(0),
automatic(true)
{
  gameInfo.playersPerTeam = numOfRobots / 2 - 2;
  gameInfo.firstHalf = 1;
  gameInfo.dropInTime = -1;
  gameInfo.secsRemaining = durationOfHalf;
  teamInfos[TEAM_BLUE].teamNumber = 1;
  teamInfos[TEAM_RED].teamNumber = 2;
  teamInfos[TEAM_RED].teamColor = TEAM_RED;
}

void GameController::registerSimulatedRobot(int robot, SimulatedRobot &simulatedRobot)
{
  ASSERT(!robots[robot].simulatedRobot);
  robots[robot].simulatedRobot = &simulatedRobot;
  if(fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}

bool GameController::handleGlobalCommand(const std::string& command)
{
  if(command == "initial")
  {
    gameInfo.state = STATE_INITIAL;
    timeOfLastDropIn = 0;
    gameInfo.secsRemaining = durationOfHalf;
    return true;
  }
  else if(command == "ready")
  {
    gameInfo.state = STATE_READY;
    for(int i = 0; i < numOfRobots; ++i)
      if(robots[i].info.penalty)
        handleRobotCommand(i, "none");
    timeWhenStateBegan = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if(command == "set")
  {
    gameInfo.state = STATE_SET;
    for(int i = 0; i < numOfRobots; ++i)
      robots[i].info.penalty = none;

    if(automatic)
    {
      placeGoalie(1);
      placeGoalie(numOfRobots / 2 + 1);
      placeDefensivePlayers(gameInfo.kickOffTeam == TEAM_BLUE ? numOfRobots / 2 + 2 : 2);
      placeOffensivePlayers(gameInfo.kickOffTeam == TEAM_BLUE ? 2 : numOfRobots / 2 + 2);
      executePlacement();
    }

    timeWhenStateBegan = SystemCall::getCurrentSystemTime();
    SimulatedRobot::moveBall(Vector3<>(0.f,0.f,50.f),true);
    return true;
  }
  else if(command == "playing")
  {
    gameInfo.state = STATE_PLAYING;
    timeWhenHalfStarted = SystemCall::getCurrentSystemTime() - (durationOfHalf - gameInfo.secsRemaining) * 1000;
    return true;
  }
  else if(command == "finished")
  {
    gameInfo.state = STATE_FINISHED;
    return true;
  }
  else if(command == "kickOffBlue")
  {
    gameInfo.kickOffTeam = TEAM_BLUE;
    return true;
  }
  else if(command == "kickOffRed")
  {
    gameInfo.kickOffTeam = TEAM_RED;
    return true;
  }
  else if(command == "outByBlue")
  {
    gameInfo.dropInTeam = TEAM_BLUE;
    timeOfLastDropIn = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if(command == "outByRed")
  {
    gameInfo.dropInTeam = TEAM_RED;
    timeOfLastDropIn = SystemCall::getCurrentSystemTime();
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
  for(int i = 0; i < numOfPenalties; ++i)
    if(command == getName((Penalty) i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (uint8_t) i;
      if(i)
      {
        r.timeWhenPenalized = SystemCall::getCurrentSystemTime();
        if(automatic)
          placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark,
                          fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
      }
      else if(automatic)
      {
        ASSERT(r.simulatedRobot);
        Vector2<> ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y >= 0 ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y >= 0 ? pi_2 : -pi_2);
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
  Vector2<> newPos(robot < numOfRobots / 2 ? x : -x, y);
  for(;;)
  {
    int j = 0;
    while(j < numOfRobots &&
      (j == robot || !robots[j].simulatedRobot || (robots[j].lastPose.translation - newPos).abs() >= 300))
      ++j;
    if(j == numOfRobots)
    {
      r.lastPose = Pose2D(rotation, newPos.x, newPos.y);
      r.simulatedRobot->moveRobot(Vector3<>(newPos.x, newPos.y, dropHeight), Vector3<>(0, 0, rotation), true);
      break;
    }
    else
      newPos.x += newPos.x < 0 ? -400 : 400;
  }
}

bool GameController::inOwnPenaltyArea(int robot) const
{
  const Robot& r = robots[robot];
  if(r.lastPose.translation.y < fieldDimensions.yPosRightPenaltyArea ||
     r.lastPose.translation.y > fieldDimensions.yPosLeftPenaltyArea)
    return false;
  else if(robot < numOfRobots / 2)
    return r.lastPose.translation.x >= fieldDimensions.xPosOpponentPenaltyArea &&
           (r.lastPose.translation.x <= fieldDimensions.xPosOpponentGroundline ||
            (r.lastPose.translation.x <= fieldDimensions.xPosOpponentGoal &&
             r.lastPose.translation.y >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y <= fieldDimensions.yPosLeftGoal));
  else
    return r.lastPose.translation.x <= fieldDimensions.xPosOwnPenaltyArea &&
           (r.lastPose.translation.x >= fieldDimensions.xPosOwnGroundline ||
            (r.lastPose.translation.x >= fieldDimensions.xPosOwnGoal &&
             r.lastPose.translation.y >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y <= fieldDimensions.yPosLeftGoal));
}

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  r.manuallyPlaced = r.simulatedRobot && !inOwnPenaltyArea(robot);
  if(r.manuallyPlaced)
    r.lastPose = robot < numOfRobots / 2 ? Pose2D(-pi, fieldDimensions.xPosOpponentGroundline, 0)
                                         : Pose2D(0, fieldDimensions.xPosOwnGroundline, 0);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2D* poses, int numOfPoses)
{
  ASSERT(numOfPoses <= numOfFieldPlayers);

  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(i != robot && robots[i].simulatedRobot)
    {
      const Robot& r2 = robots[i];
      float minDistance = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for(int j = 0; j < numOfPoses; ++j)
      {
        const Pose2D& pose = poses[j];
        float distance = (pose.translation - r2.lastPose.translation).abs();
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
  while(i < numOfPoses && occupied[i])
    ++i;
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot)
{
  static const Pose2D poses[2][numOfFieldPlayers] =
  {
    {
      Pose2D(0, -fieldDimensions.centerCircleRadius - footLength, 0),
      Pose2D(0, fieldDimensions.xPosOwnPenaltyMark, fieldDimensions.yPosRightGoal),
      Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftPenaltyArea),
      Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightPenaltyArea)
    },
    {
      Pose2D(-pi, fieldDimensions.centerCircleRadius + footLength, 0),
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosLeftGoal),
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftPenaltyArea),
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightPenaltyArea)
    }
  };

  // Move all field players that are not in their own half or in their penalty area.
  // Count robots in center circle
  int numOfRobotsInCenterCircle = 0;
  float minDistance = std::numeric_limits<float>::max();
  int indexOfMinDistance = 0;
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = false;
    r.manuallyPlaced = r.simulatedRobot &&
                       (inOwnPenaltyArea(i) ||
                        r.lastPose.translation.y < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 && (r.lastPose.translation.x < footLength ||
                                                 r.lastPose.translation.x > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x > -footLength ||
                                                  r.lastPose.translation.x < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0], numOfFieldPlayers);
    else
    {
      float distance = r.lastPose.translation.abs();
      if(distance < fieldDimensions.centerCircleRadius + footLength)
      {
        ++numOfRobotsInCenterCircle;
        if(distance < minDistance)
        {
          minDistance = distance;
          indexOfMinDistance = i;
        }
      }
    }
  }

  // If there is more than one robot in center circle, keep the closest
  // to the field center and move the others away.
  if(numOfRobotsInCenterCircle > 1)
    for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    {
      Robot& r = robots[i];
      if(i != indexOfMinDistance && r.lastPose.translation.abs() < fieldDimensions.centerCircleRadius + 120.f)
        placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0] + 1, numOfFieldPlayers - 1);
    }
}

void GameController::placeDefensivePlayers(int minRobot)
{
  static const Pose2D poses[2][numOfFieldPlayers] =
  {
    {
      Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
      Pose2D(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)
    },
    {
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
      Pose2D(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)
    }
  };
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.simulatedRobot &&
                       (inOwnPenaltyArea(i) ||
                        r.lastPose.translation.abs() < fieldDimensions.centerCircleRadius + footLength ||
                        r.lastPose.translation.y < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 && (r.lastPose.translation.x < footLength ||
                                                 r.lastPose.translation.x > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x > -footLength ||
                                                  r.lastPose.translation.x < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0], numOfFieldPlayers);
  }
}

void GameController::executePlacement()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    const Robot& r = robots[i];
    if(r.manuallyPlaced)
      r.simulatedRobot->moveRobot(Vector3<>(r.lastPose.translation.x, r.lastPose.translation.y, dropHeight),
                          Vector3<>(0, 0, r.lastPose.rotation), true);
  }
}

void GameController::referee()
{
  if(automatic)
  {
    SYNC;
    switch(gameInfo.state)
    {
    case STATE_READY:
      if(SystemCall::getTimeSince(timeWhenStateBegan) < 2000)
        timeWhenLastRobotMoved = 0;
      if(SystemCall::getTimeSince(timeWhenStateBegan) >= 45000 ||
         (timeWhenLastRobotMoved && SystemCall::getTimeSince(timeWhenLastRobotMoved) > 2000))
        handleGlobalCommand("set");
      break;

    case STATE_SET:
      if(SystemCall::getTimeSince(timeWhenStateBegan) >= 5000)
        handleGlobalCommand("playing");
      break;

    case STATE_PLAYING:
      switch(updateBall())
      {
      case GOAL_BY_BLUE:
        ++teamInfos[TEAM_BLUE].score;
        VERIFY(handleGlobalCommand("kickOffRed"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case GOAL_BY_RED:
        ++teamInfos[TEAM_RED].score;
        VERIFY(handleGlobalCommand("kickOffBlue"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case OUT_BY_BLUE:
        VERIFY(handleGlobalCommand("outByBlue"));
        break;
      case OUT_BY_RED:
        VERIFY(handleGlobalCommand("outByRed"));
        break;
      case NONE:
        break;
      }
    }
  }
}

GameController::BallOut GameController::updateBall()
{
  BallOut result = NONE;
  Vector2<> ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  if(!fieldDimensions.isInsideField(ballPos))
  {
    if(fabs(ballPos.y) < fieldDimensions.yPosLeftGoal) // goal
    {
      result = ballPos.x > fieldDimensions.xPosOpponentGroundline ? GOAL_BY_RED : GOAL_BY_BLUE;
    }
    else
    {
      float x;
      if((Pose2D(ballPos) - lastBallContactPose).translation.x > 0) // 1m behind robot
        x = (lastBallContactPose + Pose2D(-1000, 0)).translation.x;
      else // 1m behind where ball went out
        x = (Pose2D(lastBallContactPose.rotation, ballPos) + Pose2D(-1000, 0)).translation.x;

      if(fabs(ballPos.x) > fieldDimensions.xPosOpponentGroundline && (Pose2D(x, 0) - Pose2D(lastBallContactPose.rotation)).translation.x > 0)
        x = 0; // center line
      else if(x < fieldDimensions.xPosOwnDropInLine)
        x = fieldDimensions.xPosOwnDropInLine; // clip
      else if(x > fieldDimensions.xPosOpponentDropInLine)
        x = fieldDimensions.xPosOpponentDropInLine; // clip
      ballPos.x = x;

      if(ballPos.y < 0)
        ballPos.y = fieldDimensions.yPosRightDropInLine; // right throw-in line
      else
        ballPos.y = fieldDimensions.yPosLeftDropInLine; // left throw-in line

      SimulatedRobot::moveBall(Vector3<>(ballPos.x, ballPos.y, 100.f), true);
      result = lastBallContactPose.rotation == 0 ? OUT_BY_RED : OUT_BY_BLUE;
    }
  }
  return result;
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2D(SimulatedRobot::isBlue(robot) ? pi : 0, SimulatedRobot::getPosition(robot));
}

void GameController::writeGameInfo(Out& stream)
{
  SYNC;
  if(timeOfLastDropIn)
    gameInfo.dropInTime = (unsigned short) (SystemCall::getTimeSince(timeOfLastDropIn) / 1000);
  else
    gameInfo.dropInTime = -1;
  if(gameInfo.state == STATE_PLAYING)
    gameInfo.secsRemaining = (uint16_t) (durationOfHalf - SystemCall::getTimeSince(timeWhenHalfStarted) / 1000);
  gameInfo.timeLastPackageReceived = SystemCall::getCurrentSystemTime();
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
  if(r.info.penalty)
    r.info.secsTillUnpenalised = (uint8_t) (std::max(int(45 - SystemCall::getTimeSince(r.timeWhenPenalized) / 1000), 0));
  Pose2D pose;
  ASSERT(r.simulatedRobot);
  r.simulatedRobot->getRobotPose(pose);
  if(robot < numOfRobots / 2)
    pose = Pose2D(pi) + pose;
  if((pose.translation - r.lastPose.translation).abs() > 5 ||
     normalize(pose.rotation - r.lastPose.rotation) > 0.05)
  {
    timeWhenLastRobotMoved = SystemCall::getCurrentSystemTime();
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
    "outByRed"
  };
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  for(int i = 0; i < numOfPenalties; ++i)
    if(i != obstruction)
      completion.insert(std::string("pr ") + getName((Penalty) i));
}
