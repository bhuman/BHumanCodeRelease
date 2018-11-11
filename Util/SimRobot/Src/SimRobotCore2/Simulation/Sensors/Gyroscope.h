/**
* @file Simulation/Sensors/Gyroscope.h
* Declaration of class Gyroscope
* @author Colin Graf
*/

#pragma once

#include "Simulation/Sensors/Sensor.h"

class Body;

/**
* @class Gyroscope
* A simulated gyroscope sensor
*/
class Gyroscope : public Sensor
{
public:
  /** Default constructor */
  Gyroscope();

private:
  /**
  * @class GyroscopeSensor
  * The gyroscope sensor interface
  */
  class GyroscopeSensor : public Sensor::Port
  {
  public:
    Body* body; /**< The body where the gyroscope is mounted on. */
    float angularVel[4]; /**< The sensor reading. */
    Pose3f offset; /**< Offset of the sensor relative to the body. */

    /** Update the sensor value. Is called when required. */
    void updateValue() override;

    //API
    bool getMinAndMax(float& min, float& max) const override {return false;}
  } sensor;

  /** Initializes the gyroscope after all attributes have been set */
  void createPhysics() override;

  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

  /** Registers this object with children, actuators and sensors at SimRobot's GUI. */
  void registerObjects() override;
};
