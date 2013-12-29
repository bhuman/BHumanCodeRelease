/**
* @file Controller/Views/SensorView.h
*
* Declaration of class SensorView
*
* @author of original sensorview <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Jeff
* @author Colin Graf
*/

#pragma once

#include "SimRobot.h"

class RobotConsole;
class SensorData;
class SensorWidget;

/**
* @class SensorView
* A class to represent a view with information about the sensor values.
*/
class SensorView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param robotConsole The robot console which owns \c sensorData and \c filteredSensorData.
  * @param sensorData A reference to the jointData representation of the robot console.
  * @param filteredSensorData A reference to the jointData representation of the robot console.
  */
  SensorView(const QString& fullName, RobotConsole& robotConsole, const SensorData& sensorData, const SensorData& filteredSensorData);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const SensorData&  sensorData; /**< The most current sensor data. */
  const SensorData& filteredSensorData; /**< A reference to the filtered sensor data of the robot console. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class SensorWidget;
};
