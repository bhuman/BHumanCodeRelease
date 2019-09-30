/**
 * @file GameInfo.cpp
 * The file implements a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author Thomas Röfer
 */

#include "GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Global.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"

GameInfo::GameInfo()
{
  memset(static_cast<RoboCup::RoboCupGameControlData*>(this), 0, sizeof(RoboCup::RoboCupGameControlData));
}

static void drawDigit(int digit, const Vector3f& pos, float size, const ColorRGBA& color)
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
  digit = digits[std::abs(digit)];
  for(int i = 0; i < 7; ++i)
    if(digit & (1 << i))
    {
      Vector3f from = pos - points[i] * size;
      Vector3f to = pos - points[i + 1] * size;
      LINE3D("representation:GameInfo", from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), 2, color);
    }
}

void GameInfo::draw() const
{
  DEBUG_DRAWING3D("representation:GameInfo", "field")
  {
    float yPosLeftSideline = 3000.f;
    if(Blackboard::getInstance().exists("FieldDimensions"))
      yPosLeftSideline = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]).yPosLeftSideline;
    yPosLeftSideline += 500;
    const int mins = std::abs(static_cast<int>(secsRemaining)) / 60;
    const int secs = std::abs(static_cast<int>(secsRemaining)) % 60;
    const ColorRGBA color = secsRemaining < 0 ? ColorRGBA::red : ColorRGBA::black;
    drawDigit(mins / 10, Vector3f(-350, yPosLeftSideline, 1000), 200, color);
    drawDigit(mins % 10, Vector3f(-80, yPosLeftSideline, 1000), 200, color);
    drawDigit(secs / 10, Vector3f(280, yPosLeftSideline, 1000), 200, color);
    drawDigit(secs % 10, Vector3f(550, yPosLeftSideline, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, yPosLeftSideline, 890, 0, yPosLeftSideline, 910, 3, color);
    LINE3D("representation:GameInfo", 0, yPosLeftSideline, 690, 0, yPosLeftSideline, 710, 3, color);

    if(secondaryTime != 0)
    {
      const int secMins = std::abs(static_cast<int>(secondaryTime)) / 60;
      const int secSecs = std::abs(static_cast<int>(secondaryTime)) % 60;
      const ColorRGBA color = secondaryTime < 0 ? ColorRGBA::red : ColorRGBA::blue;
      drawDigit(secMins / 10, Vector3f(-175, yPosLeftSideline, 500), 100, color);
      drawDigit(secMins % 10, Vector3f(-40, yPosLeftSideline, 500), 100, color);
      drawDigit(secSecs / 10, Vector3f(140, yPosLeftSideline, 500), 100, color);
      drawDigit(secSecs % 10, Vector3f(275, yPosLeftSideline, 500), 100, color);
      LINE3D("representation:GameInfo", 0, yPosLeftSideline, 445, 0, yPosLeftSideline, 455, 3, color);
      LINE3D("representation:GameInfo", 0, yPosLeftSideline, 345, 0, yPosLeftSideline, 355, 3, color);
    }
  }

  DEBUG_DRAWING("representation:GameInfo", "drawingOnField")
  {
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    const char* sign = secsRemaining < 0 ? "-" : "";
    const int mins = std::abs(static_cast<int>(secsRemaining)) / 60;
    const int secs = std::abs(static_cast<int>(secsRemaining)) % 60;
    const std::string secsAsString = std::to_string(secs);
    DRAWTEXT("representation:GameInfo", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 500, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, "Time remaining: " << sign << mins << ":" << (secs < 10 ? "0" + secsAsString : secsAsString));
    DRAWTEXT("representation:GameInfo", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 300, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, (firstHalf ? "First" : "Second") << " half");
    DRAWTEXT("representation:GameInfo", xPosOwnFieldBorder + 1700, yPosRightFieldBorder + 300, (xPosOwnFieldBorder / -5200.f) * 180, ColorRGBA::white, "State: " << getStateAsString());
  }
}

std::string GameInfo::getStateAsString() const
{
  switch(state)
  {
    case STATE_INITIAL:
      return "Initial";
    case STATE_READY:
      return "Ready";
    case STATE_SET:
      return "Set";
    case STATE_PLAYING:
      switch(setPlay)
      {
        case SET_PLAY_NONE:
          return "Playing";
        case SET_PLAY_GOAL_FREE_KICK:
          return "Goal Free Kick";
        case SET_PLAY_PUSHING_FREE_KICK:
          return "Pushing Free Kick";
        case SET_PLAY_CORNER_KICK:
          return "Corner Kick";
        case SET_PLAY_KICK_IN:
          return "Kick In";
        default:
          return "Unknown";
      }
    case STATE_FINISHED:
      return "Finished";
    default:
      return "Unknown";
  }
}

void GameInfo::serialize(In* in, Out* out)
{
  STREAM(packetNumber);
  STREAM(competitionPhase); // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  STREAM(competitionType);  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_MIXEDTEAM)
  STREAM(gamePhase); // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(setPlay); // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_FREE_KICK, etc)
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickingTeam); // team number
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(secondaryTime);
  STREAM(timeLastPacketReceived); // used to decide whether a gameController is running
}

void GameInfo::reg()
{
  PUBLISH(reg);
  REG_CLASS(GameInfo);
  REG(packetNumber);
  REG(competitionPhase);
  REG(competitionType);
  REG(gamePhase);
  REG(state);
  REG(setPlay);
  REG(firstHalf);
  REG(kickingTeam);
  REG(secsRemaining);
  REG(secondaryTime);
  REG(timeLastPacketReceived);
}
