/**
 * @file Modules/Infrastructure/RobotHealthProvider.cpp
 * This file implements a module that provides information about the robot's health.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "RobotHealthProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Debugging/Annotation.h"
#include "Framework/Settings.h"
#include "Streaming/Global.h"
#include "Streaming/InStreams.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#ifdef TARGET_ROBOT
#include <unistd.h>
#endif

void RobotHealthProvider::update(RobotHealth& robotHealth)
{
  // Transfer information from other thread:
  robotHealth = theMotionRobotHealth;
  // Compute frame rate of cognition thread:
  unsigned now = Time::getCurrentSystemTime();
  if(lastExecutionTime != 0)
    timeBuffer.push_front(now - lastExecutionTime);
  robotHealth.cognitionFrameRate = timeBuffer.sum() ? 1000.0f / timeBuffer.averagef() : 0.f;
  lastExecutionTime = now;

  std::string wavName = Global::getSettings().headName.c_str();
  wavName.append(".wav");

  // read cpu and main board temperature
  robotHealth.cpuTemperature = theSystemSensorData.cpuTemperature == SensorData::off ? 0 : static_cast<unsigned char>(theSystemSensorData.cpuTemperature);
  if(robotHealth.cpuTemperature > cpuHeat && theFrameInfo.getTimeSince(highCPUTemperatureSince) / 1000 > 20)
  {
    highCPUTemperatureSince = theFrameInfo.time;
    if(enableName)
      SystemCall::playSound(wavName.c_str());
    SystemCall::say("CPU temperature at exclamation mark", theLibDemo.isDemoActive);
  }
#ifdef TARGET_ROBOT
  if(theFrameInfo.getTimeSince(lastWlanCheckedTime) > 10 * 1000)
  {
    lastWlanCheckedTime = theFrameInfo.time;
    robotHealth.wlan = access("/sys/class/net/wlan0", F_OK) == 0;
  }
#endif

  // MotionRobotHealthProvider filters over 30 entries. If one motion frame was dropped then check 31 frames later again
  if(theFrameInfo.getTimeSince(lastMotionFrameDropTimestamp) > 31.f * Global::getSettings().motionCycleTime * 1000.f && robotHealth.maxMotionTime > 1.5f * Global::getSettings().motionCycleTime * 1000.f)
  {
    lastMotionFrameDropTimestamp = theFrameInfo.time;
    ANNOTATION("RobotHealthProvider", "Motion frame missing: " << robotHealth.maxMotionTime);
  }

  if(theFrameInfo.getTimeSince(lastRelaxedHealthComputation) > 5000)
  {
    lastRelaxedHealthComputation = theFrameInfo.time;

    // transfer maximal temperature, battery level and total current from SensorData:
    robotHealth.batteryLevel = static_cast<unsigned char>((theSystemSensorData.batteryLevel == SensorData::off ? 1.f : theSystemSensorData.batteryLevel) * 100.f);
    robotHealth.maxJointTemperatureStatus = *std::max_element(theJointSensorData.status.begin(), theJointSensorData.status.end());
    robotHealth.jointWithMaxTemperature = static_cast<Joints::Joint>(std::distance(theJointSensorData.temperatures.begin(),
                                                                                   std::max_element(theJointSensorData.temperatures.begin(),
                                                                                                    theJointSensorData.temperatures.end())));
    robotHealth.totalCurrent = std::accumulate(theJointSensorData.currents.begin(), theJointSensorData.currents.end(), 0.0f);

    // Add cpu load, memory load and robot name:
    float memoryUsage, load[3];
    SystemCall::getLoad(memoryUsage, load);
    robotHealth.load[0] = static_cast<unsigned char>(load[0] * 10.f);
    robotHealth.load[1] = static_cast<unsigned char>(load[1] * 10.f);
    robotHealth.load[2] = static_cast<unsigned char>(load[2] * 10.f);
    robotHealth.memoryUsage = static_cast<unsigned char>(memoryUsage * 100.f);
    robotHealth.robotName = Global::getSettings().headName;

    //battery warning
    if(lastBatteryLevel < robotHealth.batteryLevel)
      batteryVoltageFalling = false;
    else if(lastBatteryLevel > robotHealth.batteryLevel)
      batteryVoltageFalling = true;
    if(robotHealth.batteryLevel < batteryLow)
    {
      if(batteryVoltageFalling && theFrameInfo.getTimeSince(startBatteryLow) > 1000)
      {
        if(enableName)
          SystemCall::playSound(wavName.c_str());
        SystemCall::say("Low battery", true);
        // next warning in 90 seconds
        startBatteryLow = theFrameInfo.time + 30000;
        batteryVoltageFalling = false;
      }
    }
    else if(startBatteryLow < theFrameInfo.time)
      startBatteryLow = theFrameInfo.time;
    lastBatteryLevel = robotHealth.batteryLevel;
    // temperature status warning
    robotHealth.maxJointTemperatureStatus = *std::max_element(theJointSensorData.status.begin(), theJointSensorData.status.end());
    if(robotHealth.maxJointTemperatureStatus > JointSensorData::regular)
    {
      if(theFrameInfo.getTimeSince(highTemperatureSince) > 1000)
      {
        if(enableName)
          SystemCall::playSound(wavName.c_str());

        unsigned timeToNextScream = timeBetweenHeatScreams;
        if(robotHealth.maxJointTemperatureStatus >= JointSensorData::TemperatureStatus::criticallyHot)
        {
          SystemCall::say("Fire exclamation mark", theLibDemo.isDemoActive);
          timeToNextScream /= 4;
        }
        else if(robotHealth.maxJointTemperatureStatus >= JointSensorData::TemperatureStatus::veryHot)
        {
          SystemCall::say("Fire", theLibDemo.isDemoActive);
          timeToNextScream /= 2;
        }
        else
        {
          SystemCall::say("Heat", theLibDemo.isDemoActive);
        }
        highTemperatureSince = theFrameInfo.time + timeToNextScream;
      }
    }
    else if(highTemperatureSince < theFrameInfo.time)
      highTemperatureSince = theFrameInfo.time;

    robotHealth.configuration = RobotHealth::Configuration::CONFIGURATION;
    robotHealth.location.assign(Global::getSettings().location);
    robotHealth.scenario.assign(Global::getSettings().scenario);
  }
}

MAKE_MODULE(RobotHealthProvider);
