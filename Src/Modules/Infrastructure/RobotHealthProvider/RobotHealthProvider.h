/**
 * @file Modules/Infrastructure/RobotHealthProvider.h
 * This file declares a module that provides information about the robot's health.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"

MODULE(RobotHealthProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(MotionRobotHealth),
  REQUIRES(SystemSensorData),
  USES(BHumanMessageOutputGenerator),
  PROVIDES(RobotHealth),
  LOADS_PARAMETERS(
  {,
    (char) batteryLow,                    /**< The voltage below which the robot gives low battery warnings. */
    (int) cpuHeat,
    (unsigned) timeBetweenHeatScreams,
    (bool) enableName,                    /**< The robots mentions its name when complaining, if true */
  }),
});

/**
 * @class RobotHealthProvider
 * A module that provides information about the robot's health
 */
class RobotHealthProvider : public RobotHealthProviderBase
{
  RingBufferWithSum<unsigned, 30> timeBuffer; /**< Buffered timestamps of previous executions */
  unsigned lastExecutionTime = 0;
  unsigned lastRelaxedHealthComputation = 0;
  unsigned startBatteryLow = 0; /**< Last time the battery state was not low. */
  float lastBatteryLevel = 1.f;
  bool batteryVoltageFalling = false;
  unsigned highTemperatureSince = 0;
  unsigned highCPUTemperatureSince = 0;
#ifdef TARGET_ROBOT
  unsigned int lastWlanCheckedTime = 0;
#endif

  /** The main function, called every cycle
   * @param robotHealth The data struct to be filled
   */
  void update(RobotHealth& robotHealth) override;
};
