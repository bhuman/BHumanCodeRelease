/**
* @file Simulation/Joints/Joint.h
* Declaration of class Joint
* @author Colin Graf
* @author Thomas Röfer
*/

#pragma once

#include <ode/ode.h>

#include "Simulation/Actuators/Actuator.h"

class Axis;

/**
* @class Joint
* A Joint joint that connects two bodies
*/
class Joint : public Actuator
{
public:
  Axis* axis;
  dJointID joint;

  /** Default constructor */
  Joint() : axis(0), joint(0) {}

  /** Destructor */
  virtual ~Joint();

private:
  /** Draws appearance primitives of the object (including children) on the currently selected OpenGL context (in order to create a display list) */
  virtual void assembleAppearances() const;

  /**
  * Draws physical primitives of the object (including children) on the currently selected OpenGL context
  * @param flags Flags to enable or disable certain features
  */
  virtual void drawPhysics(unsigned int flags) const;

  /** Registers this object with children, actuators and sensors at SimRobot's GUI */
  virtual void registerObjects();
};
