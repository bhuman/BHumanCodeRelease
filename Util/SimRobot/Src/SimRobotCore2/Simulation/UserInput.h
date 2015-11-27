/**
 * @file Simulation/UserInput.h
 * Declaration of class to forward user input. An actuator port can be set
 * by the user and a controller can read that value through a sensor port.
 * @author Thomas Röfer
 */

#pragma once

#include "Simulation/SimObject.h"
#include <QStringList>

class UserInput : public SimObject, public SimRobot::Object
{
public:
  class InputPort : public SimRobotCore2::ActuatorPort
  {
  public:
    QString fullName; /**< The path name to the object in the scene graph */
    QString unit; /**< The unit of the input's value */
    float min; /**< The minimum value */
    float max; /**< The maximum value */
    float defaultValue; /**< The value returned when this user input is turned off. */
    SimRobotCore2::SensorPort::Data data; /**< The current value */

    // API
    virtual bool getMinAndMax(float& min, float& max) const {min = this->min; max = this->max; return true;}
    virtual const QString& getFullName() const {return fullName;}
    virtual const QIcon* getIcon() const;
    virtual SimRobot::Widget* createWidget();
    virtual const QString& getUnit() const {return unit;}
    virtual void setValue(float value);
  } inputPort;

private:
  class OutputPort : public SimRobotCore2::SensorPort
  {
  public:
    const InputPort* input; /**< The input actuator port */
    QList<int> dimensions; /**< Dummy dimensions to return. */
    QStringList descriptions; /**< Dummy descriptions to return. */

    // API
    virtual const QString& getFullName() const {return input->fullName;}
    virtual const QIcon* getIcon() const;
    virtual SimRobot::Widget* createWidget();
    virtual const QList<int>& getDimensions() const {return dimensions;}
    virtual const QStringList& getDescriptions() const {return descriptions;}
    virtual const QString& getUnit() const {return input->unit;}
    virtual SensorType getSensorType() const {return SensorType::floatSensor;}
    virtual Data getValue() {return input->data;}
    virtual bool renderCameraImages(SimRobotCore2::SensorPort** cameras, unsigned int count) {return false;}
    virtual bool getMinAndMax(float& min, float& max) const {return input->getMinAndMax(min, max);}
  } outputPort;

  /** Registers this object with children, actuators and sensors at SimRobot's GUI */
  virtual void registerObjects();

  // API
  virtual const QString& getFullName() const {return SimObject::getFullName();}
  virtual SimRobot::Widget* createWidget() {return SimObject::createWidget();}
  virtual const QIcon* getIcon() const;
};