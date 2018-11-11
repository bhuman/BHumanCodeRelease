/**
* @file Simulation/Actuator.h
* Declaration of class Actuator
* @author Colin Graf
*/

#pragma once

#include <QString>

#include "Simulation/PhysicalObject.h"

/**
* @class Actuator
* An abstract class for actuators
*/
class Actuator : public PhysicalObject, public SimRobotCore2::Actuator
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
    const QString& getFullName() const override {return fullName;}
    const QIcon* getIcon() const override;
    SimRobot::Widget* createWidget() override;
    const QString& getUnit() const override {return unit;}
  };

private:
  /**
  * Registers an element as parent
  * @param element The element to register
  */
  void addParent(Element& element) override;

private:
  // API
  const QString& getFullName() const override {return SimObject::getFullName();}
  SimRobot::Widget* createWidget() override {return SimObject::createWidget();}
  const QIcon* getIcon() const override {return SimObject::getIcon();}
  SimRobotCore2::Renderer* createRenderer() override {return SimObject::createRenderer();}
  bool registerDrawing(SimRobotCore2::Controller3DDrawing& drawing) override {return ::PhysicalObject::registerDrawing(drawing);}
  bool unregisterDrawing(SimRobotCore2::Controller3DDrawing& drawing) override {return ::PhysicalObject::unregisterDrawing(drawing);}
  SimRobotCore2::Body* getParentBody() override {return ::PhysicalObject::getParentBody();}
};
