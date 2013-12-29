/**
* @file Simulation/Sensor.h
* Declaration of class Sensor
* @author Colin Graf
*/

#pragma once

#include <QStringList>

#include "Simulation/SimObject.h"
#include "Simulation/PhysicalObject.h"

/**
* @class Sensor
* An abstract class for sensors
*/
class Sensor : public PhysicalObject, public SimRobotCore2::Sensor
{
public:
  class Port : public SimRobotCore2::SensorPort
  {
  public:
    QString fullName; /**< The path name to the object in the scene graph */
    SensorType sensorType; /**< The data type of the sensor readings */
    Data data; /**< The sensor reading */
    QList<int> dimensions; /**< The dimensions of the sensor readings */
    QStringList descriptions; /**< A description for each sensor reading dimension */
    QString unit; /**< The unit of the sensor readings */
    unsigned int lastSimulationStep; /**< The last time this sensor was computed. */

    /** Default constructor */
    Port() : lastSimulationStep(0xffffffff) {}

     /** Update the sensor value. Is called when required. */
    virtual void updateValue() = 0;

  private:
    // API
    virtual const QString& getFullName() const {return fullName;}
    virtual const QIcon* getIcon() const;
    virtual SimRobot::Widget* createWidget();
    virtual const QList<int>& getDimensions() const {return dimensions;}
    virtual const QStringList& getDescriptions() const {return descriptions;}
    virtual const QString& getUnit() const {return unit;}
    virtual SensorType getSensorType() const {return sensorType;}
    virtual Data getValue();
    virtual bool renderCameraImages(SimRobotCore2::SensorPort** cameras, unsigned int count) {return false;}
  };

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
