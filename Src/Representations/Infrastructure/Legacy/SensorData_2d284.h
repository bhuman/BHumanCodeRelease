/**
* @file Representations/Infrastructure/Legacy/SensorData_2d284.h
*
* This file declares a class to represent the sensor data received from the robot.
*
* @author <A href="mailto:Harlekin@tzi.de">Philippe Schober</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"

/**
* @class SensorData_2d284
* A class to represent the sensor data received from the robot.
*/
class SensorData_2d284 : public Streamable
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
    fsrLFL,
    fsrLFR,
    fsrLBL,
    fsrLBR,
    fsrRFL,
    fsrRFR,
    fsrRBL,
    fsrRBR,
    usL,
    usR,
    angleX,
    angleY
  );

  enum
  {
    off = JointData::off /**< A special value to indicate that the sensor is missing. */
  };

  ENUM(UsActuatorMode,
    leftToLeft,
    leftToRight,
    rightToLeft,
    rightToRight
  );

  float data[numOfSensors]; /**< The data of all sensors. */
  short currents[JointData::numOfJoints]; /**< The currents of all motors. */
  unsigned char temperatures[JointData::numOfJoints]; /**< The temperature of all motors. */
  unsigned timeStamp; /**< The time when the sensor data was received. */

  UsActuatorMode usActuatorMode; /**< The ultrasonice measure method which was used for measuring \c data[usL] and \c data[usR]. */
  unsigned usTimeStamp; /**< The time when the ultrasonic measurements were taken. */

  /**
  * Default constructor.
  */
  SensorData_2d284() : timeStamp(0), usActuatorMode(leftToLeft), usTimeStamp(0)
  {
    for(int i = 0; i < numOfSensors; ++i)
      data[i] = off;
    for(int i = 0; i < JointData::numOfJoints; ++i)
      currents[i] = temperatures[i] = 0;
  }

private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read.
  * @param out The stream to which the object is written.
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(data);
    STREAM(currents);
    STREAM(temperatures);
    STREAM(timeStamp);
    STREAM(usActuatorMode);
    STREAM(usTimeStamp);
    STREAM_REGISTER_FINISH;
  }
};
