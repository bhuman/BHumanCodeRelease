/**
 * @file Modules/Infrastructure/MotionRobotHealthProvider.h
 * This file declares a module that provides information about the robot's health.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Yannik Meinken
 */

#pragma once

#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"

MODULE(MotionRobotHealthProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(RawInertialSensorData),
  PROVIDES(MotionRobotHealth),
  DEFINES_PARAMETERS(
  {,
    (float)(1.5f) droppedFactorThreshold, /**< Factor that the time between motion frames must be higher than the expected value to assume a frame was dropped in between. */
    (int)(10000) bodyDisconnectWarningTime, /**< Repeat the sound every 10secs. */
    (unsigned)(3) multipleFramesDroppedThreshold, /**< Number of frames that must be dropped in a row to consider it as multiple frames. */
    (float)(0.1) bodyDisconnectThreshold, /**< If the time between the last motion frame and the current is more then this (in seconds) we consider it a body disconnect. 100ms*/
    (unsigned)(1) executionTimesThreshold, /**< Only after this number of executions body disconnect will be detected. */
  }),
});

/**
 * @class MotionRobotHealthProvider
 * A module that provides information about the robot's health
 */
class MotionRobotHealthProvider : public MotionRobotHealthProviderBase
{
private:
  /** The main function, called every cycle
   * @param motionRobotHealth The data struct to be filled
   */
  void update(MotionRobotHealth& motionRobotHealth) override;

  RingBufferWithSum<unsigned, 30> timeBuffer; /**< Buffered timestamps of previous executions */
  unsigned lastExecutionTime;
  unsigned lastFrameTime;
  unsigned executionTimes;

  unsigned bodyDisconnectTimestamp = 0;

  RawInertialSensorData lastInertialSensorData;

public:
  /** Constructor. */
  MotionRobotHealthProvider() : lastExecutionTime(0), lastFrameTime(0), executionTimes(0) {}
};
