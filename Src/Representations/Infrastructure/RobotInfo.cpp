/**
 * @file RobotInfo.h
 * The file declares a class that encapsulates the structure RobotInfo defined in
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

void RobotInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(number); // robot number: 1..11
  STREAM(penalty); // PENALTY_NONE, PENALTY_BALL_HOLDING, ...
  STREAM(secsTillUnpenalised); // estimate of time till unpenalised.
  STREAM_REGISTER_FINISH;
}

RobotInfo::RobotInfo() : number(&Global::getSettings() ? Global::getSettings().playerNumber : 0)
{
  memset((RoboCup::RobotInfo*) this, 0, sizeof(RoboCup::RobotInfo));
}
