/*
 * SensorDataRev5703.h
 *
 *  Created on: Jun 5, 2010
 *      Author: reich
 */

#pragma once

class SensorDataRev5703 : public Streamable
{
protected:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(data);
    STREAM(currents);
    STREAM(temperatures);
    STREAM(usSensorType);
    STREAM(timeStamp);
    STREAM_REGISTER_FINISH;
  }

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
    us,
    angleX,
    angleY
  );

  ENUM(UsSensorType,
    left,
    leftToRight,
    rightToLeft,
    right
  );

  float data[numOfSensors]; /**< The data of all sensors. */
  UsSensorType usSensorType; /**< The ultrasonice measure method which was used for measuring \c data[us]. */
  short currents[JointData::numOfJoints]; /**< The currents of all motors. */
  unsigned char temperatures[JointData::numOfJoints]; /**< The temperature of all motors. */
  unsigned timeStamp; /**< The time when the sensor data was received. */
};
