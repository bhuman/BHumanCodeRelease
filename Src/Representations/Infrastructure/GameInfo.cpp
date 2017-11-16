/**
 * @file GameInfo.cpp
 * The file implements a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    DRAWTEXT("representation:GameInfo", xPosOwnFieldBorder + 200,  yPosRightFieldBorder + 500, (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, "Time remaining: " << (int)(secsRemaining / 60) << ":" << (secsRemaining % 60));
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
  STREAM(packetNumber);
  STREAM(gameType); // type of the game (GAME_ROUNDROBIN, GAME_PLAYOFF, GAME_DROPIN)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickOffTeam); // team number
  STREAM(secondaryState);  // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  STREAM(dropInTeam); // team number
  STREAM(dropInTime); // number of seconds passed since the last drop in. -1 before first dropin.
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(secondaryTime);
  STREAM(timeLastPackageReceived); // used to decide whether a gameController is running
  STREAM_REGISTER_FINISH;
}

void RawGameInfo::operator >> (BHumanMessage& m) const
{
  m.theBHULKsStandardMessage.gameControlData.timestampWhenReceived = timeLastPackageReceived;

  m.theBHULKsStandardMessage.gameControlData.packetNumber = packetNumber;
  m.theBHULKsStandardMessage.gameControlData.gameType = gameType;
  m.theBHULKsStandardMessage.gameControlData.state = state;
  m.theBHULKsStandardMessage.gameControlData.firstHalf = firstHalf;
  m.theBHULKsStandardMessage.gameControlData.kickOffTeam = kickOffTeam;
  m.theBHULKsStandardMessage.gameControlData.secondaryState = secondaryState;
  m.theBHULKsStandardMessage.gameControlData.dropInTeam = dropInTeam;
  m.theBHULKsStandardMessage.gameControlData.dropInTime = dropInTime;
  m.theBHULKsStandardMessage.gameControlData.secsRemaining = secsRemaining;
  m.theBHULKsStandardMessage.gameControlData.secondaryTime = secondaryTime;
}

void RawGameInfo::operator<< (const BHumanMessage& m)
{
  timeLastPackageReceived = m.toLocalTimestamp(m.theBHULKsStandardMessage.gameControlData.timestampWhenReceived);
  (RoboCup::RoboCupGameControlData&)(*this) << m.theBHULKsStandardMessage.gameControlData;
}

void RoboCup::operator<<(RoboCupGameControlData r, const B_HULKs::OwnTeamInfo& bhOti)
{
  r.packetNumber = bhOti.packetNumber;
  r.gameType = bhOti.gameType;
  r.state = bhOti.state;
  r.firstHalf = bhOti.firstHalf;
  r.kickOffTeam = bhOti.kickOffTeam;
  r.secondaryState = bhOti.secondaryState;
  r.dropInTeam = bhOti.dropInTeam;
  r.dropInTime = bhOti.dropInTime;
  r.secsRemaining = bhOti.secsRemaining;
  r.secondaryTime = bhOti.secondaryTime;

  const int ownTeamArrayPos = r.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1;

  r.teams[ownTeamArrayPos].score = bhOti.score;
  for(int i = 0; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
    r.teams[ownTeamArrayPos].players[i].penalty = bhOti.playersArePenalized[i] ? PENALTY_MANUAL : PENALTY_NONE;
}
