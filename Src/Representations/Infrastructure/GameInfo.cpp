/**
* @file GameInfo.cpp
* The file implements a class that encapsulates the structure RoboCupGameControlData
* defined in the file RoboCupGameControlData.h that is provided with the GameController.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "GameInfo.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Vector3.h"
#include <cstring>

GameInfo::GameInfo() : timeLastPackageReceived(0)
{
  memset((RoboCup::RoboCupGameControlData*) this, 0, sizeof(RoboCup::RoboCupGameControlData));
}

void GameInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickOffTeam); // TEAM_BLUE, TEAM_RED
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(dropInTeam); // TEAM_BLUE, TEAM_RED
  STREAM(dropInTime); // number of seconds passed since the last drop in. -1 before first dropin.
  STREAM(secondaryState);  // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  STREAM(timeLastPackageReceived) // used to decide wether a gameController is running
  STREAM_REGISTER_FINISH;
}

#ifndef RELEASE
static void drawDigit(int digit, const Vector3<>& pos, float size, const ColorRGBA& color)
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
  digit = digits[std::abs(digit)];
  for(int i = 0; i < 7; ++i)
    if(digit & (1 << i))
    {
      Vector3<> from = pos - points[i] * size;
      Vector3<> to = pos - points[i + 1] * size;
      LINE3D("representation:GameInfo", from.x, from.y, from.z, to.x, to.y, to.z, 2, color);
    }
}
#endif

void GameInfo::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:GameInfo", "field",
  {
    int mins = std::abs((int) (short) secsRemaining) / 60;
    int secs = std::abs((int) (short) secsRemaining) % 60;
    ColorRGBA color = (short) secsRemaining < 0 ? ColorRGBA(255, 0, 0) : ColorRGBA();
    drawDigit(mins / 10, Vector3<>(-350, 3500, 1000), 200, color);
    drawDigit(mins % 10, Vector3<>(-80, 3500, 1000), 200, color);
    drawDigit(secs / 10, Vector3<>(280, 3500, 1000), 200, color);
    drawDigit(secs % 10, Vector3<>(550, 3500, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, 3500, 890, 0, 3500, 910, 3, color);
    LINE3D("representation:GameInfo", 0, 3500, 690, 0, 3500, 710, 3, color);
  });

  DECLARE_DEBUG_DRAWING("representation:GameInfo","drawingOnField",
  {
    DRAWTEXT("representation:GameInfo", -5000, -3200, 200, ColorClasses::white, "Time remaining: " << (int)(secsRemaining/60) << ":" << (secsRemaining%60));
    DRAWTEXT("representation:GameInfo", -5000, -3400, 200, ColorClasses::white, (firstHalf ? "First" : "Second") << " half");
    std::string stateStr;
    switch(state)
    {
    case STATE_INITIAL:
      stateStr = "Initial";
      break;
    case STATE_READY:
      stateStr = "Ready";
      break;
    case STATE_SET:
      stateStr = "Set";
      break;
    case STATE_PLAYING:
      stateStr = "Playing";
      break;
    case STATE_FINISHED:
      stateStr = "Finished";
      break;
    default:
      stateStr = "Unknown";
    }

    DRAWTEXT("representation:GameInfo", -3500, 2200, 14, ColorClasses::white, "State: " << stateStr);
  });
}
