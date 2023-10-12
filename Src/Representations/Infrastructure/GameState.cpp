/**
 * @file GameState.cpp
 *
 * This file implements methods of the \c GameState representation.
 *
 * @author Arne Hasselbring
 */

#include "GameState.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Debugging/ColorRGBA.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Blackboard.h"
#include "Framework/Settings.h"
#include "Math/Eigen.h"
#include "Streaming/Global.h"
#include <algorithm>

static void drawChar(int character, const Vector3f& pos, float size, [[maybe_unused]] const ColorRGBA& color)
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
  if(character >= 0 && character < static_cast<char>(sizeof(chars)))
  {
    character = chars[character];
    for(int i = 0; i < 7; ++i)
      if(character & (1 << i))
      {
        Vector3f from = pos - points[i] * size;
        Vector3f to = pos - points[i + 1] * size;
        LINE3D("representation:GameState", from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), 2, color);
      }
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

GameState::GameState() :
  playerNumber(Global::settingsExist() ? Global::getSettings().playerNumber : 0)
{
  if(Global::settingsExist())
  {
    ownTeam.number = Global::getSettings().teamNumber;
    ownTeam.fieldPlayerColor = Global::getSettings().fieldPlayerColor;
    ownTeam.goalkeeperColor = Global::getSettings().goalkeeperColor;
  }
  opponentTeam.fieldPlayerColor = ownTeam.fieldPlayerColor;
  opponentTeam.goalkeeperColor = ownTeam.goalkeeperColor;
  do
  {
    opponentTeam.fieldPlayerColor = static_cast<Team::Color>((static_cast<unsigned>(opponentTeam.fieldPlayerColor) + 1) % Team::Color::numOfTeamColors);
    opponentTeam.goalkeeperColor = static_cast<Team::Color>((static_cast<unsigned>(opponentTeam.goalkeeperColor) + 1) % Team::Color::numOfTeamColors);
  }
  while(opponentTeam.fieldPlayerColor == ownTeam.fieldPlayerColor || opponentTeam.fieldPlayerColor == ownTeam.goalkeeperColor || opponentTeam.goalkeeperColor == ownTeam.fieldPlayerColor);
}

void GameState::draw() const
{
  DEBUG_DRAWING3D("representation:GameState", "field")
  {
    float yPosLeftSideline = 3000.f;
    if(Blackboard::getInstance().exists("FieldDimensions"))
      yPosLeftSideline = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]).yPosLeftSideline;
    yPosLeftSideline += 500;

    std::string gameState = "INITIAL";
    if(isReady())
      gameState = "READY";
    else if(isSet())
      gameState = "SET";
    else if(isPlaying())
      gameState = "PLAYING";
    else if(isFinished())
      gameState = "FINISHED";
    print(gameState, Vector3f(static_cast<float>((gameState.size()) * -135.f + 35.f) / 2.f + 100.f, yPosLeftSideline, 500), 100, ColorRGBA::blue);

    std::string setPlay;
    if(isPushingFreeKick())
      setPlay = "PUSHING";
    else if(isGoalKick())
      setPlay = "GOAL";
    else if(isCornerKick())
      setPlay = "CORNER";
    else if(isKickIn())
      setPlay = "KICKIN";
    else if(isPenaltyKick())
      setPlay = "PENALTY";
    print(setPlay, Vector3f(-1635.f, yPosLeftSideline, 500), 100, ColorRGBA::blue);

    if(Blackboard::getInstance().exists("FrameInfo"))
    {
      const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
      if(timeWhenPhaseEnds)
      {
        const int secsRemaining = std::min(std::abs(theFrameInfo.getTimeSince(timeWhenPhaseEnds)) / 1000, 5999);
        const int mins = secsRemaining / 60;
        const int secs = secsRemaining % 60;
        const ColorRGBA color = timeWhenPhaseEnds < theFrameInfo.time ? ColorRGBA::red : ColorRGBA::black;
        print(std::to_string(mins / 10) + std::to_string(mins % 10), Vector3f(-350, yPosLeftSideline, 1000), 200, color);
        print(std::to_string(secs / 10) + std::to_string(secs % 10), Vector3f(280, yPosLeftSideline, 1000), 200, color);
        LINE3D("representation:GameState", 0, yPosLeftSideline, 890, 0, yPosLeftSideline, 910, 3, color);
        LINE3D("representation:GameState", 0, yPosLeftSideline, 690, 0, yPosLeftSideline, 710, 3, color);
      }

      if(timeWhenStateEnds)
      {
        const int secondaryTime = std::min(std::abs(theFrameInfo.getTimeSince(timeWhenStateEnds)) / 1000, 5999);
        const int mins = secondaryTime / 60;
        const int secs = secondaryTime % 60;
        const ColorRGBA color = timeWhenStateEnds < theFrameInfo.time ? ColorRGBA::red : ColorRGBA::blue;
        print(std::to_string(mins / 10) + std::to_string(mins % 10), Vector3f(1285, yPosLeftSideline, 500), 100, color);
        print(std::to_string(secs / 10) + std::to_string(secs % 10), Vector3f(1600, yPosLeftSideline, 500), 100, color);
        LINE3D("representation:GameState", 1460, yPosLeftSideline, 445, 1460, yPosLeftSideline, 455, 3, color);
        LINE3D("representation:GameState", 1460, yPosLeftSideline, 345, 1460, yPosLeftSideline, 355, 3, color);
      }
    }

    const auto drawScore = [&](const Team& team, bool ownTeam)
    {
      const float x = ownTeam ? -1535.f : 1465.f;
      const int score = std::min(static_cast<int>(team.score), 99);
      print(std::to_string(score / 10) + std::to_string(score % 10), Vector3f(x, yPosLeftSideline, 1000), 200, ColorRGBA::fromTeamColor(team.fieldPlayerColor));
      std::string budget = "   " + std::to_string(team.messageBudget);
      print(budget.substr(budget.size() - 4), Vector3f(x - 135.f, yPosLeftSideline, 1300), 100, ColorRGBA::fromTeamColor(team.fieldPlayerColor));
    };

    drawScore(ownTeam, true);
    drawScore(opponentTeam, false);
  }

  DEBUG_DRAWING3D("representation:GameState:playerNumber", "robot")
  {
    const float centerDigit = (playerNumber > 1) ? 50.f : 0;
    int num = playerNumber;
    ROTATE3D("representation:GameState:playerNumber", 0, 0, pi_2);
    DRAWDIGIT3D("representation:GameState:playerNumber", num, Vector3f(centerDigit, 0.f, 500.f), 80, 5, ColorRGBA::green);
  }

  DEBUG_DRAWING3D("representation:GameState:background", "field")
  {
    QUAD3D("representation:GameState:background", Vector3f(-1850, 3510, 200), Vector3f(1850, 3510, 200),
           Vector3f(1850, 3510, 1400), Vector3f(-1850, 3510, 1400), ColorRGBA(255, 255, 255, 128));
  }

  DEBUG_DRAWING("representation:GameState", "drawingOnField")
  {
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    if(Blackboard::getInstance().exists("FrameInfo") && timeWhenPhaseEnds)
    {
      const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
      const int secsRemaining = std::abs(theFrameInfo.getTimeSince(timeWhenPhaseEnds)) / 1000;
      const char* sign = timeWhenPhaseEnds < theFrameInfo.time ? "-" : "";
      const int mins = std::abs(secsRemaining) / 60;
      const int secs = std::abs(secsRemaining) % 60;
      const std::string secsAsString = std::to_string(secs);
      DRAW_TEXT("representation:GameState", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 500, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, "Time remaining: " << sign << mins << ":" << (secs < 10 ? "0" + secsAsString : secsAsString));
    }
    DRAW_TEXT("representation:GameState", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 300, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, (phase == firstHalf ? "First" : "Second") << " half");
    DRAW_TEXT("representation:GameState", xPosOwnFieldBorder + 1700, yPosRightFieldBorder + 300, (xPosOwnFieldBorder / -5200.f) * 180, ColorRGBA::white, "State: " << TypeRegistry::getEnumName(state));
  }

  DEBUG_DRAWING("representation:GameState:ownTeam", "drawingOnField")
  {
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    DRAW_TEXT("representation:GameState:ownTeam", xPosOwnFieldBorder + 200, yPosRightFieldBorder - 100, (xPosOwnFieldBorder / -5200.f) * 140, ColorRGBA::red, "Team color: " << TypeRegistry::getEnumName(ownTeam.fieldPlayerColor) << "/" << TypeRegistry::getEnumName(ownTeam.goalkeeperColor));
  }
}
