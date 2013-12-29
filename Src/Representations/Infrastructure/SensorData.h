/**
* @file Representations/Infrastructure/SensorData.h
*
* This file declares a class to represent the sensor data received from the robot.
*
* @author <A href="mailto:Harlekin@tzi.de">Philippe Schober</A>
*/

#pragma once

#include "JointData.h"

/**
* @class SensorData
* A class to represent the sensor data received from the robot.
*/
STREAMABLE(SensorData,
{
public:
  ENUM(Sensor,
    gyroX,
    gyroY,
    gyroZ,
    accX,
    accY,
    accZ,
    batteryLevel,
    fsrLFL,     //the feetsensors of the Nao-Robot
    fsrLFR,
    fsrLBL,
    fsrLBR,
    fsrRFL,
    fsrRFR,
    fsrRBL,
    fsrRBR,
    usL,
    usL1,
    usL2,
    usL3,
    usL4,
    usL5,
    usL6,
    usL7,
    usL8,
    usL9,
    usLEnd,
    us = usLEnd,
    usR = us,
    us1,
    usR1 = us1,
    us2,
    usR2 = us2,
    us3,
    usR3 = us3,
    us4,
    usR4 = us4,
    us5,
    usR5 = us5,
    us6,
    usR6 = us6,
    us7,
    usR7 = us7,
    us8,
    usR8 = us8,
    us9,
    usR9 = us9,
    usREnd,
    angleX = usREnd,
    angleY
  );

  enum {off = JointData::off}; /**< A special value to indicate that the sensor is missing. */

  // Inserted dummies, due to access based on array index. (http://www.aldebaran-robotics.com/documentation/naoqi/sensors/dcm/pref_file_architecture.html#us-actuator-value)
  ENUM(UsActuatorMode,
    leftToLeft,
    leftToRight,
    rightToLeft,
    rightToRight,
    numOfSingleUsActuatorModes,
    bothToSame = numOfSingleUsActuatorModes,
    bothToOther
  ),

  (float[numOfSensors]) data, /**< The data of all sensors. */
  (short[JointData::numOfJoints]) currents, /**< The currents of all motors. */
  (unsigned char[JointData::numOfJoints]) temperatures, /**< The temperature of all motors. */
  (unsigned)(0) timeStamp, /**< The time when the sensor data was received. */
  (UsActuatorMode)(leftToLeft) usActuatorMode, /**< The ultrasonice measure method which was used for measuring \c data[usL] and \c data[usR]. */
  (unsigned)(0) usTimeStamp, /**< The time when the ultrasonic measurements were taken. */

  // Initialization
  for(int i = 0; i < numOfSensors; ++i)
    data[i] = off;
  for(int i = 0; i < JointData::numOfJoints; ++i)
    currents[i] = temperatures[i] = 0;
});

/**
* A class to represent filtered sensor data.
*/
class FilteredSensorData : public SensorData {};
