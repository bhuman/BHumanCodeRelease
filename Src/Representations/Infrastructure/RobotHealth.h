/**
* @file RobotHealth.h
* The file declares two classes to transport information about the current robot health
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <cstring>

/**
* @class MotionRobotHealth
* All information collected within motion process.
*/
STREAMABLE(MotionRobotHealth,
{,
  (float)(0) motionFrameRate, /**< Frames per second within process "Motion" */
  (float) avgMotionTime, /**< average execution time */
  (float) maxMotionTime, /**< Maximum execution time */
  (float) minMotionTime, /**< Minimum execution time */
});

/**
* @class RobotHealth
* Full information about the robot.
*/
STREAMABLE_WITH_BASE(RobotHealth, MotionRobotHealth,
{
public:
  /**
   * Configurations that can be deployed-
   * Note that they must start with an uppercase letter.
   */
  ENUM(Configuration,
    Debug,
    Develop,
    Release
  );

  /** Default constructor. */
  RobotHealth()
  {
    load[0] = load[1] = load[2] = 0;
    strncpy(hash, "unknown", sizeof(hash));
    strncpy(location, "unknown", sizeof(location));
  }

  /** Assigning MotionRobotHealth
  * @param motionRobotHealth Information from the motion process
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
    DECLARE_PLOT("representation:RobotHealth:batteryLevel");
    PLOT("representation:RobotHealth:batteryLevel", batteryLevel);
    DECLARE_PLOT("representation:RobotHealth:maxJointTemperature");
    PLOT("representation:RobotHealth:maxJointTemperature", maxJointTemperature);
    DECLARE_PLOT("representation:RobotHealth:totalCurrent");
    PLOT("representation:RobotHealth:totalCurrent", totalCurrent);
  },

  (float)(0.f) cognitionFrameRate, /**< Frames per second within process "Cognition" */
  (unsigned char)(0) batteryLevel, /**< Current batteryLevel of robot battery in percent */
  (float)(0.f) totalCurrent, /**< Sum of all motor currents ( as a measure for the robot's current load) */
  (unsigned char)(0) maxJointTemperature, /**< Highest temperature of a robot actuator */
  (JointData, Joint)(HeadYaw) jointWithMaxTemperature, /**< The hottest joint. */
  (unsigned char)(0) cpuTemperature, /**< The temperature of the cpu */
  (unsigned char[3]) load, /**< load averages */
  (unsigned char)(0) memoryUsage, /**< Percentage of used memory */
  (std::string) robotName, /**< For fancier drawing :-) */
  (unsigned)(0) ballPercepts, /**< A ball percept counter used to determine ball percepts per hour */
  (unsigned)(0) linePercepts, /**< A line percept counter used to determine line percepts per hour */
  (unsigned)(0) goalPercepts, /**< A goal percept counter used to determine goal percepts per hour */
  (bool)(true) wlan, /**< Status of the wlan hardware. true: wlan hardware is ok. false: wlan hardware is (probably physically) broken. */
  (Configuration)(Develop) configuration, /**< The configuration that was deployed. */
  (char[5]) hash, /**< The first 5 digits of the hash of the git HEAD that was deployed. */
  (bool)(false) clean, /**< Was the working copy clean when it was deployed? */
  (char[3]) location, /**< The first 3 letters of the location selected. */
});
