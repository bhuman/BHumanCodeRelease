/**
* @file Simulation/Sensors/Accelerometer.h
* Declaration of class Accelerometer
* @author Colin Graf
*/

#pragma once

#include "Simulation/Sensors/Sensor.h"

class Body;

/**
* @class Accelerometer
* A simulated accelerometer
*/
class Accelerometer : public Sensor
{
public:
  /** Default constructor */
  Accelerometer();

private:
  /**
  * @class AccelerometerSensor
  * The acceleration sensor interface
  */
  class AccelerometerSensor : public Sensor::Port
  {
  public:
    Body* body; /** The body were the accelerometer is mounted on */
    float linearVelInWorld[4]; /** The linear velocity of the body where the accelerometer is mounted on */
    float linearAcc[4]; /** The sensor reading */

    /** Default constructor */
    AccelerometerSensor()
    {
      linearVelInWorld[0] = linearVelInWorld[1] = linearVelInWorld[2] = 0.f;
    }

    /** Update the sensor value. Is called when required. */
    virtual void updateValue();

    //API
    virtual bool getMinAndMax(float& min, float& max) const {return false;}
  } sensor;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

  /** Registers this object with children, actuators and sensors at SimRobot's GUI */
  virtual void registerObjects();
};
