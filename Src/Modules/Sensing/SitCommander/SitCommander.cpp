/**
 * @file SitCommander.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "SitCommander.h"
#include "Tools/Motion/SensorData.h"

MAKE_MODULE(SitCommander, sensing)

void SitCommander::update(SitCommand& sitCommand)
{
  if(!(sitCommand.gettingCommanded = (theRobotInfo.hasFeature(RobotInfo::headLEDs) && theHeadAngleRequest.pan != SensorData::off)))
    return;

  if(theKeyStates.pressed[KeyStates::headMiddle] || theKeyStates.pressed[KeyStates::headRear] || theKeyStates.pressed[KeyStates::headFront])
  {
    if(sitCommand.changing >= 0)
      sitCommand.changing = std::min(1.f, sitCommand.changing + Constants::motionCycleTime / holdingButtonTime);
    else
      sitCommand.changing = 0.f;

    if(sitCommand.changing >= 1.f && !changedBefore)
    {
      sitCommand.command = sitCommand.command == SitCommand::playExclamationMark ? SitCommand::sitExclamationMark : SitCommand::playExclamationMark;
      changedBefore = true;
    }
  }
  else
  {
    sitCommand.changing = -1.f;
    changedBefore = false;
  }
}
