/**
* @file Controller/Views/JointView.h
* Declaration of class for displaying the requested and measured joint angles.
* @author Colin Graf
*/

#pragma once

#include "SimRobot.h"

class RobotConsole;
class JointData;
class JointRequest;
class SensorData;
class JointWidget;

/**
* @class JointView
* A class implements a DirectView for displaying the requested and measured joint angles.
*/
class JointView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph.
  * @param robotConsole The robot console which owns \c jointData, \c sensorData and \c jointRequest.
  * @param jointData A reference to the jointData representation of the robot console.
  * @param sensorData A reference to the sensorData representation of the robot console.
  * @param jointRequest A reference to the jointRequest representation of the robot console.
  */
  JointView(const QString& fullName, RobotConsole& robotConsole, const JointData& jointData,
    const SensorData& sensorData, const JointData& jointRequest);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const JointData& jointData; /**< A reference to the jointData representation of the robot console. */
  const SensorData& sensorData; /**< A reference to the sensorData representation of the robot console. */
  const JointData& jointRequest; /**< A reference to the jointRequest representation of the robot console. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class JointWidget;
};
