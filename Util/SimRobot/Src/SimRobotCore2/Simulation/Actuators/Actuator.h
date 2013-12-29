/**
* @file Simulation/Actuator.h
* Declaration of class Actuator
* @author Colin Graf
*/

#pragma once

#include <QString>

#include "Simulation/PhysicalObject.h"
#include "Simulation/GraphicalObject.h"

/**
* @class Actuator
* An abstract class for actuators
*/
class Actuator : public PhysicalObject, public GraphicalObject, public SimRobotCore2::Actuator
{
public:
  class Port : public SimRobotCore2::ActuatorPort
  {
  public:
    QString fullName; /**< The path name to the object in the scene graph */
    QString unit; /**< The unit of the actuator's setpoint */

    /** Called before computing a simulation step to do something with the set-point of the actuator */
    virtual void act() = 0;

  private:
    // API
    virtual const QString& getFullName() const {return fullName;}
    virtual const QIcon* getIcon() const;
    virtual SimRobot::Widget* createWidget();
    virtual const QString& getUnit() const {return unit;}
  };

private:
  /**
  * Registers an element as parent
  * @param element The element to register
  */
  virtual void addParent(Element& element);

private:
  // API
  virtual const QString& getFullName() const {return SimObject::getFullName();}
  virtual SimRobot::Widget* createWidget() {return SimObject::createWidget();}
  virtual const QIcon* getIcon() const {return SimObject::getIcon();}
  virtual SimRobotCore2::Renderer* createRenderer() {return SimObject::createRenderer();}
  virtual bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing) {return ::PhysicalObject::registerDrawing(drawing);}
  virtual bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing) {return ::PhysicalObject::unregisterDrawing(drawing);}
  virtual SimRobotCore2::Body* getParentBody() {return ::PhysicalObject::getParentBody();}
};
