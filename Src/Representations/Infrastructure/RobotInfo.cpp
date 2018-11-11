/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#include "RobotInfo.h"
#include <cstring>
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/BHAssert.h"

#ifdef TARGET_ROBOT
#include "Platform/Nao/NaoBody.h"
#endif

#include <iostream>

RobotInfo::RobotInfo() :
  number(Global::settingsExist() ? Global::getSettings().playerNumber : 0)
{
  memset((RoboCup::RobotInfo*) this, 0, sizeof(RoboCup::RobotInfo));

#ifdef TARGET_ROBOT
  NaoBody naoBody;
  headVersion = naoBody.getHeadVersion();
  bodyVersion = naoBody.getBodyVersion();
  headType = naoBody.getHeadType();
  bodyType = naoBody.getBodyType();

  std::cout << "headVersion: " << TypeRegistry::getEnumName(headVersion)
            << ", headType: " << TypeRegistry::getEnumName(headType)
            << ", bodyVersion: " << TypeRegistry::getEnumName(bodyVersion)
            << ", bodyType: " << TypeRegistry::getEnumName(bodyType) << std::endl;
#endif
}

bool RobotInfo::hasFeature(const RobotFeature feature) const
{
  switch(feature)
  {
    case hands:
    case wristYaws:
    case tactileHandSensores:
      return bodyType >= H25;
    case tactileHeadSensores:
    case headLEDs:
      return headType >= H25;
    case grippyFingers:
      return bodyType >= H25 && bodyVersion >= V5;
    case zAngle:
      return bodyVersion >= V5;
    case zGyro:
      return bodyVersion >= V5;
    default:
    {
      FAIL("Unknown feature.");
      return false;
    }
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
    case PENALTY_SPL_ILLEGAL_DEFENDER: return "Illegal Defender";
    case PENALTY_SPL_LEAVING_THE_FIELD: return "Leaving the Field";
    case PENALTY_SPL_KICK_OFF_GOAL: return "Kickoff Goal";
    case PENALTY_SPL_REQUEST_FOR_PICKUP: return "Request for Pickup";
    case PENALTY_SPL_LOCAL_GAME_STUCK: return "Local Game Stuck";
    default: return "None";
  }
}

void RobotInfo::serialize(In* in, Out* out)
{
  STREAM(number); // robot number: 1..11
  STREAM(headVersion);
  STREAM(headType);
  STREAM(bodyVersion);
  STREAM(bodyType);
  STREAM(penalty); // PENALTY_NONE, PENALTY_BALL_HOLDING, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalised.
}

void RobotInfo::reg()
{
  PUBLISH(reg);
  REG_CLASS(RobotInfo);
  REG(number);
  REG(headVersion);
  REG(headType);
  REG(bodyVersion);
  REG(bodyType);
  REG(penalty);
  REG(secsTillUnpenalised);
}
