/**
* @file Simulation/Axis.h
* Declaration of class Axis
* @author Colin Graf
*/

#pragma once

class Motor;

#include "Parser/Element.h"

class Joint;

/**
* @class Axis
* An axis of a joint
*/
class Axis : public Element
{
public:
  class Deflection
  {
  public:
    float min;
    float max;
    float stopCFM;
    float stopERP;
    float offset;

    /** Default constructor */
    Deflection() : min(0), max(0), stopCFM(-1.f), stopERP(-1.f), offset(0.f) {}
  };

  float x;
  float y;
  float z;
  float cfm;
  Deflection* deflection;
  Motor* motor;
  Joint* joint; /**< The joint that own this axis. */

  /** Default constructor */
  Axis() :  x(0), y(0), z(0), cfm(-1.f), deflection(0), motor(0), joint(0) {}

  /** Destructor */
  ~Axis();

  /** Normalizes the axis */
  void create();

private:
  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;
};
