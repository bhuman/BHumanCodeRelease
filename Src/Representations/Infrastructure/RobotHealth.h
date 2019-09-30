/**
 * @file RobotHealth.h
 * The file declares two classes to transport information about the current robot health
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/RobotParts/Joints.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include <cstring>

/**
 * @struct MotionRobotHealth
 * All information collected within motion thread.
 */
STREAMABLE(MotionRobotHealth,
{,
  (float)(0) motionFrameRate, /**< Frames per second within thread "Motion" */
  (float)(0) avgMotionTime, /**< average execution time */
  (float)(0) maxMotionTime, /**< Maximum execution time */
  (float)(0) minMotionTime, /**< Minimum execution time */
});

/**
 * @struct RobotHealth
 * Full information about the robot.
 */
STREAMABLE_WITH_BASE(RobotHealth, MotionRobotHealth, COMMA public PureBHumanArbitraryMessageParticle<idRobotHealth>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;
  /**
   * Configurations that can be deployed-
   * Note that they must start with an uppercase letter.
   */
  ENUM(Configuration,
  {,
    Debug,
    Develop,
    Release,
  });

  RobotHealth()
  {
    load[0] = load[1] = load[2] = 0;
  }

  /**
   * Assigning MotionRobotHealth
   * @param motionRobotHealth Information from the motion thread
   */
  void operator=(const MotionRobotHealth& motionRobotHealth)
  {
    motionFrameRate = motionRobotHealth.motionFrameRate;
    minMotionTime = motionRobotHealth.minMotionTime;
    maxMotionTime = motionRobotHealth.maxMotionTime;
    avgMotionTime = motionRobotHealth.avgMotionTime;
  }

  void draw() const
  {
    PLOT("representation:RobotHealth:batteryLevel", batteryLevel);
    PLOT("representation:RobotHealth:maxJointTemperature", maxJointTemperatureStatus);
    PLOT("representation:RobotHealth:totalCurrent", totalCurrent);
  },

  (float)(0.f)                                   cognitionFrameRate,        /**< Frames per second within thread "Cognition" */
  (unsigned char)(100)                           batteryLevel,              /**< Current batteryLevel of robot battery in percent */
  (float)(0.f)                                   totalCurrent,              /**< Sum of all motor currents (as a measure for the robot's current load) */
  (JointSensorData::TemperatureStatus)(JointSensorData::regular) maxJointTemperatureStatus, /**< Highest temperature status of a robot actuator */
  (Joints::Joint)(Joints::headYaw)               jointWithMaxTemperature,   /**< The hottest joint. */
  (unsigned char)(0)                             cpuTemperature,            /**< The temperature of the cpu */
  (std::array<unsigned char, 3>)                 load,                      /**< cpu load averages */
  (unsigned char)(0)                             memoryUsage,               /**< Percentage of used memory */
  (bool)(true)                                   wlan,                      /**< Status of the wlan hardware. true: wlan hardware is ok. false: wlan hardware is (probably physically) broken. */
  (std::string)                                  robotName,                 /**< For fancier drawing :-) */
  (Configuration)(Develop)                       configuration,             /**< The configuration that was deployed. */
  (std::string)                                  location,                  /**< The location selected. */
  (std::string)                                  scenario,                  /**< The scenario selected. */
});
