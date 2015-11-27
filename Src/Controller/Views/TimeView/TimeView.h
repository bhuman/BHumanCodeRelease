/**
* @file Controller/Views/TimeView.h
*
* Declaration of class TimeView
*
* @author Colin Graf
*/

#pragma once

#include <SimRobot.h>
#include <unordered_map>
#include <string>
#include <QIcon>

class RobotConsole;
class TimeInfo;
class QWidget;
class NumberTableWidgetItem;
class QTableWidgetItem;
class QTableWidget;
/**
* @class TimeView
*
* A class to represent a view with information about the timing of modules.
*
* @author Colin Graf
*/
class TimeView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph
  * @param console The console object.
  * @param info The timing info object to be visualized.
  */
  TimeView(const QString& fullName, RobotConsole& console, const TimeInfo& info);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const TimeInfo& info; /**< The Time info structure. */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class TimeWidget;//TimeWidget needs access to console
};
