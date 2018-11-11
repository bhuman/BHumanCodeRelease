/**
 * @file SitCommander.h
 *
 * This file declares a module that provides if a robot has to sit or not.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/SitCommand.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Tools/Module/Module.h"

MODULE(SitCommander,
{,
  USES(HeadAngleRequest),
  REQUIRES(KeyStates),
  REQUIRES(RobotInfo),
  PROVIDES(SitCommand),
  DEFINES_PARAMETERS(
  {,
    (float)(1.2f) holdingButtonTime,
  }),
});

class SitCommander : public SitCommanderBase
{
  void update(SitCommand& sitCommand) override;
private:
  bool changedBefore = false;
};
