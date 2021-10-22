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
#include <algorithm>

GameInfo::GameInfo()
{
  memset(static_cast<RoboCup::RoboCupGameControlData*>(this), 0, sizeof(RoboCup::RoboCupGameControlData));
}

static void drawChar(int character, const Vector3f& pos, float size, const ColorRGBA& color)
{
  static const Vector3f points[8] =
  {
    Vector3f(1, 0, 1), //  1
    Vector3f(1, 0, 0), // 0 2
    Vector3f(0, 0, 0), //  6
    Vector3f(0, 0, 1), // 5 3
    Vector3f(0, 0, 2), //  4
    Vector3f(1, 0, 2),
    Vector3f(1, 0, 1),
    Vector3f(0, 0, 1)
  };
  static const unsigned char chars[43] =
  {
    0x3f, // 0
    0x0c, // 1
    0x76, // 2
    0x5e, // 3
    0x4d, // 4
    0x5b, // 5
    0x7b, // 6
    0x0e, // 7
    0x7f, // 8
    0x5f, // 9
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0x6f, // A
    0x7f, // B
    0x33, // C
    0x3f, // D
    0x73, // E
    0x63, // F
    0x7b, // G
    0x6d, // H
    0x0c, // I
    0x3c, // J
    0x6d, // K
    0x31, // L
    0x2f, // M
    0x6d, // N
    0x3f, // O
    0x67, // P
    0x3f, // Q
    0x6f, // R
    0x5b, // S
    0x0e, // T
    0x3d, // U
    0x3d, // V
    0x3d, // W
    0x6d, // X
    0x4d, // Y
    0x76, // Z
  };
  character = chars[character];
  for(int i = 0; i < 7; ++i)
    if(character & (1 << i))
    {
      Vector3f from = pos - points[i] * size;
      Vector3f to = pos - points[i + 1] * size;
      LINE3D("representation:GameInfo", from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), 2, color);
    }
}

static void print(const std::string& text, const Vector3f& pos, float size, const ColorRGBA& color)
{
  Vector3f charPos = pos;
  for(char character : text)
  {
    drawChar(character - '0', charPos, size, color);
    charPos.x() += size * 1.35f;
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

    static std::string gameStates[] =
    {
      "INITIAL",
      "READY",
      "SET",
      "PLAYING",
      "FINISHED"
    };
    print(gameStates[state], Vector3f(static_cast<float>((gameStates[state].size()) * -135.f + 35.f) / 2.f + 100.f, yPosLeftSideline, 500), 100, ColorRGBA::blue);

    static std::string setPlays[] =
    {
      "",
      "GOAL",
      "PUSHING",
      "CORNER",
      "KICKIN",
      "PENALTY"
    };
    print(setPlays[setPlay], Vector3f(-1635.f, yPosLeftSideline, 500), 100, ColorRGBA::blue);

    const int secsRemaining = std::min(std::abs(static_cast<int>(this->secsRemaining)), 5999);
    const int mins = secsRemaining / 60;
    const int secs = secsRemaining % 60;
    const ColorRGBA color = this->secsRemaining < 0 ? ColorRGBA::red : ColorRGBA::black;
    print(std::to_string(mins / 10) + std::to_string(mins % 10), Vector3f(-350, yPosLeftSideline, 1000), 200, color);
    print(std::to_string(secs / 10) + std::to_string(secs % 10), Vector3f(280, yPosLeftSideline, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, yPosLeftSideline, 890, 0, yPosLeftSideline, 910, 3, color);
    LINE3D("representation:GameInfo", 0, yPosLeftSideline, 690, 0, yPosLeftSideline, 710, 3, color);

    if(secondaryTime != 0)
    {
      const int secondaryTime = std::min(std::abs(static_cast<int>(this->secondaryTime)), 5999);
      const int mins = secondaryTime / 60;
      const int secs = secondaryTime % 60;
      const ColorRGBA color = this->secondaryTime < 0 ? ColorRGBA::red : ColorRGBA::blue;
      print(std::to_string(mins / 10) + std::to_string(mins % 10), Vector3f(1285, yPosLeftSideline, 500), 100, color);
      print(std::to_string(secs / 10) + std::to_string(secs % 10), Vector3f(1600, yPosLeftSideline, 500), 100, color);
      LINE3D("representation:GameInfo", 1460, yPosLeftSideline, 445, 1460, yPosLeftSideline, 455, 3, color);
      LINE3D("representation:GameInfo", 1460, yPosLeftSideline, 345, 1460, yPosLeftSideline, 355, 3, color);
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
    DRAW_TEXT("representation:GameInfo", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 500, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, "Time remaining: " << sign << mins << ":" << (secs < 10 ? "0" + secsAsString : secsAsString));
    DRAW_TEXT("representation:GameInfo", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 300, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, (firstHalf ? "First" : "Second") << " half");
    DRAW_TEXT("representation:GameInfo", xPosOwnFieldBorder + 1700, yPosRightFieldBorder + 300, (xPosOwnFieldBorder / -5200.f) * 180, ColorRGBA::white, "State: " << getStateAsString());
  }
}

std::string GameInfo::getStateAsString() const
{
  switch(state)
  {
    case STATE_INITIAL:
      return "Initial";
    case STATE_READY:
      if(setPlay == SET_PLAY_PENALTY_KICK)
        return "Ready (Penalty Kick)";
      else
        return "Ready";
    case STATE_SET:
      if(setPlay == SET_PLAY_PENALTY_KICK)
        return "Set (Penalty Kick)";
      else
        return "Set";
    case STATE_PLAYING:
      switch(setPlay)
      {
        case SET_PLAY_NONE:
          return "Playing";
        case SET_PLAY_GOAL_KICK:
          return "Goal Kick";
        case SET_PLAY_PUSHING_FREE_KICK:
          return "Pushing Free Kick";
        case SET_PLAY_CORNER_KICK:
          return "Corner Kick";
        case SET_PLAY_KICK_IN:
          return "Kick In";
        case SET_PLAY_PENALTY_KICK:
          return "Penalty Kick";
        default:
          return "Unknown";
      }
    case STATE_FINISHED:
      return "Finished";
    default:
      return "Unknown";
  }
}

void GameInfo::read(In& stream)
{
  STREAM(packetNumber);
  STREAM(competitionPhase); // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  STREAM(competitionType);  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_GENERAL_PENALTY_KICK)
  STREAM(gamePhase); // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(setPlay); // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickingTeam); // team number
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(secondaryTime);
  STREAM(timeLastPacketReceived); // used to decide whether a gameController is running
}

void GameInfo::write(Out& stream) const
{
  STREAM(packetNumber);
  STREAM(competitionPhase); // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  STREAM(competitionType);  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_GENERAL_PENALTY_KICK)
  STREAM(gamePhase); // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(setPlay); // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
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
