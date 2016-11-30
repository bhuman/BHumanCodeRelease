/**
 * @file Modules/Infrastructure/RobotHealthProvider.h
 * This file declares a module that provides information about the robot's health.
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#ifdef TARGET_ROBOT
#include "Platform/Nao/NaoBody.h"
#endif

MODULE(RobotHealthProvider,
{,
  REQUIRES(BallPercept),
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(FieldLines),
  REQUIRES(MotionRobotHealth),
  REQUIRES(SystemSensorData),
  PROVIDES(RobotHealth),
  LOADS_PARAMETERS(
  {,
    (char) batteryLow,                    /**< The voltage below which the robot gives low battery warnings. */
    (int) temperatureHeat,                /**< The temperature that makes the robot complaining about the temperature. */
    (int) temperatureFire,                /**< The temperature that makes the robot complaining stronger about the temperature. */
    (int) temperatureFireExclamationMark, /**< The temperature ... */
    (int) cpuHeat,
    (bool) enableName,                    /**< The robots mentions its name when complaining, if true */
  }),
});

/**
 * @class RobotHealthProvider
 * A module that provides information about the robot's health
 */
class RobotHealthProvider : public RobotHealthProviderBase
{
public:
  /** Constructor. */
  RobotHealthProvider();

private:
  STREAMABLE(BuildInfo,
  {,
    ((RobotHealth) Configuration)(Develop) configuration, /**< The configuration that was deployed. */
    (std::string)("unknown") hash, /**< The first 5 digits of the hash of the git HEAD that was deployed. */
    (bool)(false) clean, /**< Was the working copy clean when it was deployed? */
  });

  BuildInfo buildInfo; /**< Information about the revision that was deployed. */
  RingBufferWithSum<unsigned, 30> timeBuffer; /**< Buffered timestamps of previous executions */
  unsigned lastExecutionTime;
  unsigned lastRelaxedHealthComputation;
  unsigned startBatteryLow; /**< Last time the battery state was not low. */
  float lastBatteryLevel;
  bool batteryVoltageFalling;
  unsigned highTemperatureSince;
  unsigned highCPUTemperatureSince = 0;
#ifdef TARGET_ROBOT
  NaoBody naoBody;
  unsigned int lastWlanCheckedTime;
#endif

  /** The main function, called every cycle
   * @param robotHealth The data struct to be filled
   */
  void update(RobotHealth& robotHealth);
};
