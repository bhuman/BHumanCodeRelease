/**
 * @file RobotInfo.cpp
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author Thomas Röfer
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#include "RobotInfo.h"
#include "Platform/BHAssert.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include <cstring>

RobotInfo::RobotInfo() :
  number(Global::settingsExist() ? Global::getSettings().playerNumber : 0)
{
  memset(static_cast<RoboCup::RobotInfo*>(this), 0, sizeof(RoboCup::RobotInfo));
}

std::string RobotInfo::getPenaltyAsString() const
{
  switch(penalty)
  {
    case PENALTY_SPL_ILLEGAL_BALL_CONTACT: return "Illegal Ball Contact";
    case PENALTY_SPL_PLAYER_PUSHING: return "Player Pushing";
    case PENALTY_SPL_ILLEGAL_MOTION_IN_SET: return "Illegal Motion in Set";
    case PENALTY_SPL_INACTIVE_PLAYER: return "Inactive Player";
    case PENALTY_SPL_ILLEGAL_DEFENDER: return "Illegal Defender";
    case PENALTY_SPL_LEAVING_THE_FIELD: return "Leaving the Field";
    case PENALTY_SPL_KICK_OFF_GOAL: return "Kickoff Goal";
    case PENALTY_SPL_REQUEST_FOR_PICKUP: return "Request for Pickup";
    case PENALTY_SPL_LOCAL_GAME_STUCK: return "Local Game Stuck";
    case PENALTY_SPL_ILLEGAL_POSITIONING: return "Illegal Positioning";
    default: return "None";
  }
}

void RobotInfo::serialize(In* in, Out* out)
{
  STREAM(number); // robot number: 1..11
  STREAM(penalty); // PENALTY_NONE, PENALTY_BALL_HOLDING, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalized.
}

void RobotInfo::reg()
{
  PUBLISH(reg);
  REG_CLASS(RobotInfo);
  REG(number);
  REG(penalty);
  REG(secsTillUnpenalised);
}
