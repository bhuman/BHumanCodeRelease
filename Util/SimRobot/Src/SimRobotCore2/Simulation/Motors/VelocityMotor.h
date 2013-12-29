/**
* @file Simulation/Motors/VelocityMotor.h
* Declaration of class VelocityMotor
* @author Colin Graf
* @author Thomas Röfer
*/

#pragma once

#include "Motor.h"
#include "Simulation/Sensors/Sensor.h"

/**
* @class VelocityMotor
* A motor for controlling the rotational speed of an axis
*/
class VelocityMotor : public Motor
{
public:
  float maxVelocity;
  float maxForce;

  /** Default constructor */
  VelocityMotor();

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

  /**
   * @class VelocitySensor
   * An velocity sensor interface
   */
  class VelocitySensor : public Sensor::Port
  {
  public:
    Joint* joint;
    float maxVelocity;

    //API
    virtual void updateValue();
    virtual bool getMinAndMax(float& min, float& max) const;
  } velocitySensor;

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
