/**
* @file Simulation/Motors/ServoMotor.h
* Declaration of class ServoMotor
* @author Colin Graf
*/

#pragma once

#include "Motor.h"
#include "Simulation/Sensors/Sensor.h"

/**
* @class ServoMotor
* A motor for controlling the angle of an axis
*/
class ServoMotor : public Motor
{
public:
  /**
  * @class Controller
  * A PID controller that controls the motor
  */
  class Controller
  {
  public:
    float p;
    float i;
    float d;

    /** Default constructor */
    Controller() : p(0), i(0), d(0), errorSum(0), lastError(0) {}

    /** Computes the controller output. Do not call this more than once per simulation step
    * @param currentPos A measured value
    * @param setpoint The desired value
    * @return The controller output
    */
    float getOutput(float currentPos, float setpoint);

  private:
    float errorSum;
    float lastError;
  };

  Controller controller; /**< A PID controller that controls the motor */
  float maxVelocity;
  float maxForce;

  /** Default constructor */
  ServoMotor();

private:
  /**
  * @class AngleSensor
  * An angle sensor interface
  */
  class PositionSensor : public Sensor::Port
  {
  public:
    ServoMotor* servoMotor;

    //API
    void updateValue() override;
    bool getMinAndMax(float& min, float& max) const override;
  } positionSensor;

  /** Last position of an angular hinge. */
  float lastPos;

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
