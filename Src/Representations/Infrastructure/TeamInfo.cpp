/**
 * @file TeamInfo.cpp
 * The file implements a class that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamInfo.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Settings.h"
#include <cstring>

/**
 * The class is a helper to be able to stream the players.
 * The global RobotInfo cannot be used, because it has an additional attribute.
 */
class PlayerInfo : public RoboCup::RobotInfo, public ImplicitlyStreamable {};

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
  memset((RoboCup::TeamInfo*) this, 0, sizeof(RoboCup::TeamInfo));
}

void TeamInfo::serialize(In* in, Out* out)
{
  PlayerInfo(&players)[4] = reinterpret_cast<PlayerInfo(&)[4]>(this->players);

  STREAM_REGISTER_BEGIN;
  STREAM(teamNumber); // unique team number
  STREAM(teamColor); // TEAM_BLUE, TEAM_RED
  STREAM(score); // team's score
  STREAM(players); // the team's players
  STREAM_REGISTER_FINISH;
}

static void drawDigit(int digit, const Vector3<>& pos, float size, int teamColor)
{
  static const Vector3<> points[8] =
  {
    Vector3<>(1, 0, 1),
    Vector3<>(1, 0, 0),
    Vector3<>(0, 0, 0),
    Vector3<>(0, 0, 1),
    Vector3<>(0, 0, 2),
    Vector3<>(1, 0, 2),
    Vector3<>(1, 0, 1),
    Vector3<>(0, 0, 1)
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
  ColorRGBA color = teamColor == TEAM_BLUE ? ColorRGBA(0, 0, 255) : ColorRGBA(255, 0, 0);
  digit = digits[std::abs(digit)];
  for(int i = 0; i < 7; ++i)
    if(digit & (1 << i))
    {
      Vector3<> from = pos - points[i] * size;
      Vector3<> to = pos - points[i + 1] * size;
      LINE3D("representation:TeamInfo", from.x, from.y, from.z, to.x, to.y, to.z, 2, color);
    }
}

void TeamInfo::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:TeamInfo", "field");
  {
    float x = teamColor == TEAM_BLUE ? -1535.f : 1465.f;
    drawDigit(score / 10, Vector3<>(x, 3500, 1000), 200, teamColor);
    drawDigit(score % 10, Vector3<>(x + 270, 3500, 1000), 200, teamColor);
  };
}

OwnTeamInfo::OwnTeamInfo()
{
  teamColor = &Global::getSettings() ? Global::getSettings().teamColour : TEAM_BLUE;
}

void OwnTeamInfo::draw() const
{
  //do base class drawing first.
  TeamInfo::draw();

  DECLARE_DEBUG_DRAWING("representation:OwnTeamInfo", "drawingOnField",
  {
    DRAWTEXT("representation:OwnTeamInfo", -5000, -3800, 140, ColorClasses::red, (teamColor == TEAM_BLUE ? "Blue" : "Red"));
  });
}

OpponentTeamInfo::OpponentTeamInfo()
{
  teamColor = 1 - (&Global::getSettings() ? Global::getSettings().teamColour : TEAM_BLUE);
}

