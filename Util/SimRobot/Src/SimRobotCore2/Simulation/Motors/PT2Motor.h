/**
* @file Simulation/Motors/ServoMotor.h
* Declaration of class PT2Motor
* @author Thomas Muender
*/

#pragma once

#include <deque>

#include "Motor.h"
#include "Simulation/Sensors/Sensor.h"

/**
* @class ServoMotor
* A motor for controlling the angle of an axis
*/
class PT2Motor : public Motor
{
public:

  float x = 0.f; /*< Intermediate value, can be related to current velocity by: x / T */

  float T = 0.f;
  float D = 0.f;
  float K = 0.f;
  float V = 0.f;
  float F = 0.f;

  /** Default constructor */
  PT2Motor();

private:
  /**
  * @class AngleSensor
  * An angle sensor interface
  */
  class PositionSensor : public Sensor::Port
  {
  public:
    Joint* joint;

    //API
    virtual void updateValue();
    virtual bool getMinAndMax(float& min, float& max) const;
  } positionSensor;

  std::deque<float> lastSetpoints;

  /**
  * Initializes the motor
  * @param joint The joint that is controlled by this motor
  */
  virtual void create(Joint* joint);

  /** Called before computing a simulation step to update the joint */
  virtual void act();

  /** Registers this object at SimRobot's GUI */
  virtual void registerObjects();

  // actuator API
  virtual void setValue(float value);
  virtual bool getMinAndMax(float& min, float& max) const;
};
