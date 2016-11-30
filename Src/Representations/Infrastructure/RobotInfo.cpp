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

  std::cout << "headVersion: " << getName(headVersion)
    << ", headType: " << getName(headType)
    << ", bodyVersion: " << getName(bodyVersion)
    << ", bodyType: " << getName(bodyType) << std::endl;
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
    case zGyro:
      return bodyVersion >= V5;
    default:
      ASSERT(false);
      return false;
  }
}

void RobotInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(number); // robot number: 1..11
  STREAM(headVersion);
  STREAM(headType);
  STREAM(bodyVersion);
  STREAM(bodyType);
  STREAM(penalty); // PENALTY_NONE, PENALTY_BALL_HOLDING, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalised.
  STREAM_REGISTER_FINISH;
}