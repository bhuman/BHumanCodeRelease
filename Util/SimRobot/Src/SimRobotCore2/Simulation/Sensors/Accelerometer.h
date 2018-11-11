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
    Body* body; /**< The body where the accelerometer is mounted on. */
    float linearVelInWorld[4]; /**< The linear velocity of the body where the accelerometer is mounted on. */
    float linearAcc[4]; /**< The sensor reading. */
    Pose3f offset; /**< Offset of the sensor relative to the body. */

    /** Default constructor */
    AccelerometerSensor()
    {
      linearVelInWorld[0] = linearVelInWorld[1] = linearVelInWorld[2] = 0.f;
    }

    /** Update the sensor value. Is called when required. */
    void updateValue() override;

    //API
    bool getMinAndMax(float& min, float& max) const override {return false;}
  } sensor;

  /** Initializes the accelerometer after all attributes have been set */
  void createPhysics() override;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

  /** Registers this object with children, actuators and sensors at SimRobot's GUI */
  void registerObjects() override;
};
