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

#include <SimRobot.h>

struct FsrSensorData;
struct InertialSensorData;
struct KeyStates;
struct SystemSensorData;
class RobotConsole;
class SensorWidget;

/**
 * @class SensorView
 * A class to represent a view with information about the sensor values.
 */
class SensorView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const FsrSensorData& fsrSensorData;
  const InertialSensorData& inertialSensorData;
  const KeyStates& keyStates;
  const SystemSensorData& systemSensorData;

  friend class SensorWidget;

public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph
   * @param robotConsole The robot console which owns \c sensorData.
   */
  SensorView(const QString& fullName, RobotConsole& robotConsole, const FsrSensorData& fsrSensorData,
             const InertialSensorData& inertialSensorData, const KeyStates& keyStates,
             const SystemSensorData& systemSensorData);

private:
  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}
};