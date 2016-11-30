/**
 * @file RobotHealth.h
 * The file declares two classes to transport information about the current robot health
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/RobotParts/Joints.h"
#include <cstring>

/**
 * @struct MotionRobotHealth
 * All information collected within motion process.
 */
STREAMABLE(MotionRobotHealth,
{,
  (float)(0) motionFrameRate, /**< Frames per second within process "Motion" */
  (float)(0) avgMotionTime, /**< average execution time */
  (float)(0) maxMotionTime, /**< Maximum execution time */
  (float)(0) minMotionTime, /**< Minimum execution time */
});

/**
 * @struct RobotHealth
 * Full information about the robot.
 */
STREAMABLE_WITH_BASE(RobotHealth, MotionRobotHealth,
{
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
    strncpy(hash, "unknown", sizeof(hash));
    strncpy(location, "unknown", sizeof(location));
  }

  /**
   * Assigning MotionRobotHealth
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
    PLOT("representation:RobotHealth:batteryLevel", batteryLevel);
    PLOT("representation:RobotHealth:maxJointTemperature", maxJointTemperature);
    PLOT("representation:RobotHealth:totalCurrent", totalCurrent);
  }

  std::string csv() const
  {
    static bool first = true;
    if(first)
    {
      first = false;
      return "motionFrameRate,avgMotionTime,maxMotionTime,minMotionTime,cognitionFrameRate,batteryLevel,totalCurrent,maxJointTemperature,jointWithMaxTeperature,cpuTemperature,load,memoryUsage,ballPercepts,linePercepts,goalPercepots,robotName\n" +
             std::to_string(motionFrameRate) + "," + std::to_string(avgMotionTime) + "," + std::to_string(maxMotionTime) + "," + std::to_string(minMotionTime) + "," + std::to_string(cognitionFrameRate) + "," + std::to_string(batteryLevel) + "," + std::to_string(totalCurrent) + "," + std::to_string(maxJointTemperature) + ", ," + std::to_string(cpuTemperature) + ", ," + std::to_string(memoryUsage) + "," + std::to_string(ballPercepts) + "," + std::to_string(linePercepts) + "," + robotName + "\n";
    }
    else
      return std::to_string(motionFrameRate) + "," + std::to_string(avgMotionTime) + "," + std::to_string(maxMotionTime) + "," + std::to_string(minMotionTime) + "," + std::to_string(cognitionFrameRate) + "," + std::to_string(batteryLevel) + "," + std::to_string(totalCurrent) + "," + std::to_string(maxJointTemperature) + ", ," + std::to_string(cpuTemperature) + ", ," + std::to_string(memoryUsage) + "," + std::to_string(ballPercepts) + "," + std::to_string(linePercepts) + "," + robotName + "\n";
  },

  (float)(0.f) cognitionFrameRate, /**< Frames per second within process "Cognition" */
  (unsigned char)(0) batteryLevel, /**< Current batteryLevel of robot battery in percent */
  (float)(0.f) totalCurrent, /**< Sum of all motor currents ( as a measure for the robot's current load) */
  (unsigned char)(0) maxJointTemperature, /**< Highest temperature of a robot actuator */
  ((Joints) Joint)(headYaw) jointWithMaxTemperature, /**< The hottest joint. */
  (unsigned char)(0) cpuTemperature, /**< The temperature of the cpu */
  (unsigned char[3]) load, /**< load averages */
  (unsigned char)(0) memoryUsage, /**< Percentage of used memory */
  (std::string) robotName, /**< For fancier drawing :-) */
  (unsigned)(0) ballPercepts, /**< A ball percept counter used to determine ball percepts per hour */
  (unsigned)(0) linePercepts, /**< A line percept counter used to determine line percepts per hour */
  (bool)(true) wlan, /**< Status of the wlan hardware. true: wlan hardware is ok. false: wlan hardware is (probably physically) broken. */
  (Configuration)(Develop) configuration, /**< The configuration that was deployed. */
  (char[5]) hash, /**< The first 5 digits of the hash of the git HEAD that was deployed. */
  (bool)(false) clean, /**< Was the working copy clean when it was deployed? */
  (char[3]) location, /**< The first 3 letters of the location selected. */
});
