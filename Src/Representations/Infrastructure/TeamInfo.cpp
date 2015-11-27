/**
 * @file TeamInfo.cpp
 * The file implements a struct that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamInfo.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Settings.h"
#include <cstring>

/**
 * The struct is a helper to be able to stream the players.
 * The global RobotInfo cannot be used, because it has an additional attribute.
 */
struct PlayerInfo : public RoboCup::RobotInfo {};

/**
 * Write a player info to a stream.
 * @param stream The stream that is written to.
 * @param playerInfo The data that is written.
 * @return The stream.
 */
Out& operator<<(Out& stream, const PlayerInfo& playerInfo)
{
  STREAM_REGISTER_BEGIN_EXT(playerInfo);
  STREAM_EXT(stream, playerInfo.penalty);
  STREAM_EXT(stream, playerInfo.secsTillUnpenalised);
  STREAM_REGISTER_FINISH;
  return stream;
}

/**
 * Read a player info from a stream.
 * @param stream The stream that is read from.
 * @param playerInfo The data that is read.
 * @return The stream.
 */
In& operator>>(In& stream, PlayerInfo& playerInfo)
{
  STREAM_REGISTER_BEGIN_EXT(playerInfo);
  STREAM_EXT(stream, playerInfo.penalty);
  STREAM_EXT(stream, playerInfo.secsTillUnpenalised);
  STREAM_REGISTER_FINISH;
  return stream;
}

TeamInfo::TeamInfo()
{
  memset(static_cast<RoboCup::TeamInfo*>(this), 0, sizeof(RoboCup::TeamInfo));
}

void TeamInfo::serialize(In* in, Out* out)
{
  PlayerInfo(&players)[4] = reinterpret_cast<PlayerInfo(&)[4]>(this->players);
  PlayerInfo& coach = reinterpret_cast<PlayerInfo&>(this->coach);
  char buf[sizeof(this->coachMessage) + 1];
  strncpy(buf, (const char*) this->coachMessage, sizeof(this->coachMessage));
  buf[sizeof(this->coachMessage)] = 0;
  std::string coachMessage = buf;

  STREAM_REGISTER_BEGIN;
  STREAM(teamNumber); // unique team number
  STREAM(teamColor); // TEAM_BLUE, TEAM_RED, TEAM_YELLOW, TEAM_BLACK
  STREAM(score); // team's score
  STREAM(coachMessage); // last coach message received
  STREAM(coach); // team's coach
  STREAM(players); // the team's players
  STREAM_REGISTER_FINISH;

  if(in)
  {
    if(coachMessage.empty())
      this->coachMessage[0] = 0;
    else
      strncpy((char*) this->coachMessage, &coachMessage[0], sizeof(this->coachMessage));
  }
}

static void drawDigit(int digit, const Vector3f& pos, float size, int teamColor)
{
  static const Vector3f points[8] =
  {
    Vector3f(1, 0, 1),
    Vector3f(1, 0, 0),
    Vector3f(0, 0, 0),
    Vector3f(0, 0, 1),
    Vector3f(0, 0, 2),
    Vector3f(1, 0, 2),
    Vector3f(1, 0, 1),
    Vector3f(0, 0, 1)
  };
  static const unsigned char digits[10] =
  {
    0x3f,
    0x0c,
    0x76,
    0x5e,
    0x4d,
    0x5b,
    0x7b,
    0x0e,
    0x7f,
    0x5f
  };
  const static ColorRGBA colors[] =
  {
    ColorRGBA::blue,
    ColorRGBA::red,
    ColorRGBA::yellow,
    ColorRGBA::black
  };

  digit = digits[std::abs(digit)];
  for(int i = 0; i < 7; ++i)
    if(digit & (1 << i))
    {
      Vector3f from = pos - points[i] * size;
      Vector3f to = pos - points[i + 1] * size;
      LINE3D("representation:TeamInfo", from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), 2, colors[teamColor]);
    }
}

void TeamInfo::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:TeamInfo", "field");
  {
    float x = teamColor == TEAM_BLUE ? -1535.f : 1465.f;
    drawDigit(score / 10, Vector3f(x, 3500, 1000), 200, teamColor);
    drawDigit(score % 10, Vector3f(x + 270, 3500, 1000), 200, teamColor);
  };
}

OwnTeamInfo::OwnTeamInfo()
{
  teamColor = Global::settingsExist() ? Global::getSettings().teamColor : TEAM_BLUE;
}

void OwnTeamInfo::draw() const
{
  //do base struct drawing first.
  TeamInfo::draw();

  DEBUG_DRAWING("representation:OwnTeamInfo", "drawingOnField")
  {
    DRAWTEXT("representation:OwnTeamInfo", -5000, -3800, 140, ColorRGBA::red, Settings::getName((Settings::TeamColor) teamColor));
  }
}

OpponentTeamInfo::OpponentTeamInfo()
{
  teamColor = 1 ^ (Global::settingsExist() ? Global::getSettings().teamColor : TEAM_BLUE);
}
