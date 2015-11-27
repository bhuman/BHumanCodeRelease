/**
 * @file GameInfo.cpp
 * The file implements a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameInfo.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"

GameInfo::GameInfo()
{
  memset((RoboCup::RoboCupGameControlData*) this, 0, sizeof(RoboCup::RoboCupGameControlData));
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
    const int mins = std::abs((int)(short)secsRemaining) / 60;
    const int secs = std::abs((int)(short)secsRemaining) % 60;
    const ColorRGBA color = (short)secsRemaining < 0 ? ColorRGBA::red : ColorRGBA::black;
    drawDigit(mins / 10, Vector3f(-350, 3500, 1000), 200, color);
    drawDigit(mins % 10, Vector3f(-80, 3500, 1000), 200, color);
    drawDigit(secs / 10, Vector3f(280, 3500, 1000), 200, color);
    drawDigit(secs % 10, Vector3f(550, 3500, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, 3500, 890, 0, 3500, 910, 3, color);
    LINE3D("representation:GameInfo", 0, 3500, 690, 0, 3500, 710, 3, color);
  }

  DEBUG_DRAWING("representation:GameInfo", "drawingOnField")
  {
    DRAWTEXT("representation:GameInfo", -5000, -3200, 200, ColorRGBA::white, "Time remaining: " << (int)(secsRemaining / 60) << ":" << (secsRemaining % 60));
    DRAWTEXT("representation:GameInfo", -5000, -3400, 200, ColorRGBA::white, (firstHalf ? "First" : "Second") << " half");
    DRAWTEXT("representation:GameInfo", -3500, -3400, 180, ColorRGBA::white, "State: " << getStateAsString());
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
      return "Playing";
    case STATE_FINISHED:
      return "Finished";
    default:
      return "Unknown";
  }
}

void GameInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(gameType); // type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickOffTeam); // team number
  STREAM(secondaryState);  // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  STREAM(dropInTeam); // team number
  STREAM(dropInTime); // number of seconds passed since the last drop in. -1 before first dropin.
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(timeLastPackageReceived) // used to decide whether a gameController is running
  STREAM_REGISTER_FINISH;
}