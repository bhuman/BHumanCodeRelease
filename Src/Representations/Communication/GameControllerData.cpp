/**
 * @file GameControllerData.cpp
 *
 * This file implements a representation that contains the data received from a GameController.
 *
 * @author Thomas Röfer
 * @author Arne Hasselbring
 */

#include "GameControllerData.h"

static_assert(GAMECONTROLLER_STRUCT_VERSION == 18);

struct RobotInfo : public RoboCup::RobotInfo
{
private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(RobotInfo);
    REG(penalty);
    REG(secsTillUnpenalised);
  }
};

/**
 * Write a robot info to a stream.
 * @param stream The stream that is written to.
 * @param robotInfo The data that is written.
 * @return The stream.
 */
Out& operator<<(Out& stream, const RobotInfo& robotInfo)
{
  STREAM_EXT(stream, robotInfo.penalty);
  STREAM_EXT(stream, robotInfo.secsTillUnpenalised);
  return stream;
}

/**
 * Read a robot info from a stream.
 * @param stream The stream that is read from.
 * @param robotInfo The data that is read.
 * @return The stream.
 */
In& operator>>(In& stream, RobotInfo& robotInfo)
{
  STREAM_EXT(stream, robotInfo.penalty);
  STREAM_EXT(stream, robotInfo.secsTillUnpenalised);
  return stream;
}

struct TeamInfo : public RoboCup::TeamInfo
{
private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(TeamInfo);
    REG(teamNumber);
    REG(fieldPlayerColor);
    REG(goalkeeperColor);
    REG(goalkeeper);
    REG(score);
    REG(penaltyShot);
    REG(singleShots);
    REG(messageBudget);
    REG(RobotInfo(&)[MAX_NUM_PLAYERS], players);
  }
};

/**
 * Write a team info to a stream.
 * @param stream The stream that is written to.
 * @param teamInfo The data that is written.
 * @return The stream.
 */
Out& operator<<(Out& stream, const TeamInfo& teamInfo)
{
  const RobotInfo(&players)[MAX_NUM_PLAYERS] = reinterpret_cast<const RobotInfo(&)[MAX_NUM_PLAYERS]>(teamInfo.players);
  STREAM_EXT(stream, teamInfo.teamNumber);
  STREAM_EXT(stream, teamInfo.fieldPlayerColor);
  STREAM_EXT(stream, teamInfo.goalkeeperColor);
  STREAM_EXT(stream, teamInfo.goalkeeper);
  STREAM_EXT(stream, teamInfo.score);
  STREAM_EXT(stream, teamInfo.penaltyShot);
  STREAM_EXT(stream, teamInfo.singleShots);
  STREAM_EXT(stream, teamInfo.messageBudget);
  STREAM_EXT(stream, players);
  return stream;
}

/**
 * Read a team info from a stream.
 * @param stream The stream that is read from.
 * @param teamInfo The data that is read.
 * @return The stream.
 */
In& operator>>(In& stream, TeamInfo& teamInfo)
{
  RobotInfo(&players)[MAX_NUM_PLAYERS] = reinterpret_cast<RobotInfo(&)[MAX_NUM_PLAYERS]>(teamInfo.players);
  STREAM_EXT(stream, teamInfo.teamNumber);
  STREAM_EXT(stream, teamInfo.fieldPlayerColor);
  STREAM_EXT(stream, teamInfo.goalkeeperColor);
  STREAM_EXT(stream, teamInfo.goalkeeper);
  STREAM_EXT(stream, teamInfo.score);
  STREAM_EXT(stream, teamInfo.penaltyShot);
  STREAM_EXT(stream, teamInfo.singleShots);
  STREAM_EXT(stream, teamInfo.messageBudget);
  STREAM_EXT(stream, players);
  return stream;
}

GameControllerData::GameControllerData()
{
  std::memset(static_cast<RoboCup::RoboCupGameControlData*>(this), 0, sizeof(RoboCup::RoboCupGameControlData));
}

void GameControllerData::read(In& stream)
{
  TeamInfo(&teams)[2] = reinterpret_cast<TeamInfo(&)[2]>(this->teams);
  STREAM(packetNumber);
  STREAM(playersPerTeam);
  STREAM(competitionPhase);
  STREAM(competitionType);
  STREAM(gamePhase);
  STREAM(state);
  STREAM(setPlay);
  STREAM(firstHalf);
  STREAM(kickingTeam);
  STREAM(secsRemaining);
  STREAM(secondaryTime);
  STREAM(teams);
  STREAM(timeLastPacketReceived);
  STREAM(isTrueData);
}

void GameControllerData::write(Out& stream) const
{
  const TeamInfo(&teams)[2] = reinterpret_cast<const TeamInfo(&)[2]>(this->teams);
  STREAM(packetNumber);
  STREAM(playersPerTeam);
  STREAM(competitionPhase);
  STREAM(competitionType);
  STREAM(gamePhase);
  STREAM(state);
  STREAM(setPlay);
  STREAM(firstHalf);
  STREAM(kickingTeam);
  STREAM(secsRemaining);
  STREAM(secondaryTime);
  STREAM(teams);
  STREAM(timeLastPacketReceived);
  STREAM(isTrueData);
}

void GameControllerData::reg()
{
  PUBLISH(reg);
  REG_CLASS(GameControllerData);
  REG(packetNumber);
  REG(playersPerTeam);
  REG(competitionPhase);
  REG(competitionType);
  REG(gamePhase);
  REG(state);
  REG(setPlay);
  REG(firstHalf);
  REG(kickingTeam);
  REG(secsRemaining);
  REG(secondaryTime);
  REG(TeamInfo(&)[2], teams);
  REG(timeLastPacketReceived);
  REG(isTrueData);
}
