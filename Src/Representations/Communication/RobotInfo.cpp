/**
 * @file RobotInfo.cpp
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 *
 * @author Thomas Röfer
 */

#include "RobotInfo.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <cstring>

RobotInfo::RobotInfo() :
  number(Global::settingsExist() ? Global::getSettings().playerNumber : 0),
  mode(SystemCall::getMode() == SystemCall::physicalRobot ? unstiff : active)
{
  memset(static_cast<RoboCup::RobotInfo*>(this), 0, sizeof(RoboCup::RobotInfo));
}

void RobotInfo::draw() const
{
  DEBUG_DRAWING3D("representation:RobotInfo", "robot")
  {
    float centerDigit = (number > 1) ? 50.f : 0;
    int num = number;
    ROTATE3D("representation:RobotInfo", 0, 0, pi_2);
    DRAWDIGIT3D("representation:RobotInfo", num, Vector3f(centerDigit, 0.f, 500.f), 80, 5, ColorRGBA::green);
  }
}

std::string RobotInfo::getPenaltyAsString() const
{
  switch(penalty)
  {
    case PENALTY_SPL_ILLEGAL_BALL_CONTACT: return "Illegal Ball Contact";
    case PENALTY_SPL_PLAYER_PUSHING: return "Player Pushing";
    case PENALTY_SPL_ILLEGAL_MOTION_IN_SET: return "Illegal Motion in Set";
    case PENALTY_SPL_INACTIVE_PLAYER: return "Inactive Player";
    case PENALTY_SPL_ILLEGAL_POSITION: return "Illegal Position";
    case PENALTY_SPL_LEAVING_THE_FIELD: return "Leaving the Field";
    case PENALTY_SPL_REQUEST_FOR_PICKUP: return "Request for Pickup";
    case PENALTY_SPL_LOCAL_GAME_STUCK: return "Local Game Stuck";
    case PENALTY_SPL_ILLEGAL_POSITION_IN_SET: return "Illegal Position in Set";
    case PENALTY_SUBSTITUTE: return "Substitute";
    case PENALTY_MANUAL: return "Manual";
    default: return "None";
  }
}

bool RobotInfo::isGoalkeeper() const
{
  return number == 1;
}

void RobotInfo::read(In& stream)
{
  STREAM(number); // robot number: 1..11
  STREAM(mode); // one of the modes
  STREAM(penalty); // PENALTY_NONE, PENALTY_ILLEGAL_BALL_CONTACT, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalized.
}

void RobotInfo::write(Out& stream) const
{
  STREAM(number); // robot number: 1..11
  STREAM(mode); // one of the modes
  STREAM(penalty); // PENALTY_NONE, PENALTY_ILLEGAL_BALL_CONTACT, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalized.
}

void RobotInfo::reg()
{
  PUBLISH(reg);
  REG_CLASS(RobotInfo);
  REG(number);
  REG(mode);
  REG(penalty);
  REG(secsTillUnpenalised);
}
