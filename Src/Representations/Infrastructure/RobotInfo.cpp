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

RobotInfo::RobotInfo() :
  number(Global::settingsExist() ? Global::getSettings().playerNumber : 0)
{
  memset((RoboCup::RobotInfo*) this, 0, sizeof(RoboCup::RobotInfo));
}

bool RobotInfo::hasFeature(const RobotFeature feature) const
{
  switch(feature)
  {
    case hands:
    case wristYaws:
    case tactileHandSensores:
      return naoBodyType >= H25;
    case tactileHeadSensores:
    case headLEDs:
      return naoHeadType >= H25;
    case grippyFingers:
      return naoBodyType >= H25 && naoVersion >= V5;
    case zGyro:
      return naoVersion >= V5;
    default:
      ASSERT(false);
      return false;
  }
}

void RobotInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(number); // robot number: 1..11
  STREAM(naoVersion);
  STREAM(naoBodyType);
  STREAM(naoHeadType);
  STREAM(penalty); // PENALTY_NONE, PENALTY_BALL_HOLDING, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalised.
  STREAM_REGISTER_FINISH;
}