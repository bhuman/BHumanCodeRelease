/**
* @file Controller/Views/JointView.h
* Declaration of class for displaying the requested and measured joint angles.
* @author Colin Graf
*/

#pragma once

#include <SimRobot.h>

class RobotConsole;
struct JointSensorData;
struct JointRequest;
class JointWidget;

/**
 * @class JointView
 * A class implements a DirectView for displaying the requested and measured joint angles.
 */
class JointView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const JointSensorData& jointSensorData; /**< A reference to the jointSensorData representation of the robot console. */
  const JointRequest& jointRequest; /**< A reference to the jointRequest representation of the robot console. */

  friend class JointWidget;

public:
  /**
   * Constructor.
   * @param fullName The path to this view in the scene graph.
   * @param robotConsole The robot console which owns \c jointSensorData and \c jointRequest.
   * @param jointSensorData A reference to the jointSensorData representation of the robot console.
   * @param jointRequest A reference to the jointRequest representation of the robot console.
   */
  JointView(const QString& fullName, RobotConsole& robotConsole, const JointSensorData& jointSensorData, const JointRequest& jointRequest);

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
