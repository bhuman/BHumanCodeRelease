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
    void updateValue() override;
    bool getMinAndMax(float& min, float& max) const override;
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
    void updateValue() override;
    bool getMinAndMax(float& min, float& max) const override;
  } velocitySensor;

  /**
  * Initializes the motor
  * @param joint The joint that is controlled by this motor
  */
  void create(Joint* joint) override;

  /** Called before computing a simulation step to update the joint */
   void act() override;

  /** Registers this object at SimRobot's GUI */
  void registerObjects() override;

  // actuator API
  void setValue(float value) override;
  bool getMinAndMax(float& min, float& max) const override;
};
