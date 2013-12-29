/**
* @file Controller/Views/FootView.h
*
* Declaration of class FootView
*
* @author Felix Wenk
*/

#pragma once

#include <QString>
#include <QIcon>

#include "SimRobot.h"

class RobotConsole;

/**
* @class FootView
*
* A class to represent a view displaying drawings in foot coordinates.
* @author Felix Wenk
*/
class FootView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param console The console object.
  * @param name The name of the view.
  */
  FootView(const QString& fullName, RobotConsole& console, const RobotBalance& robotBalance, const std::string& name);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const RobotBalance& robotBalance; /**< Reference to the robot balance information sent by the robot code. */
  const std::string name; /**< The name of the view. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class FootWidget;
};
