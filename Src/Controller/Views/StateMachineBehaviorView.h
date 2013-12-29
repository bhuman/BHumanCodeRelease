/**
* @file Controller/Views/StateMachineBehaviorView.h
* Declaration of class StateMachineBehaviorView
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#pragma once

#include "SimRobot.h"

class RobotConsole;
class ActivationGraph;

/**
* @class StateMachineBehaviorView
* A class to represent a view with information about a StateMachineBehavior behavior.
*/
class StateMachineBehaviorView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param console The console object.
  * @param info The StateMachineBehavior info object to be visualized.
  * @param type what to display
  */
  StateMachineBehaviorView(const QString& fullName, RobotConsole& console, const ActivationGraph& activationGraph, const unsigned& timeStamp);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const ActivationGraph& activationGraph; /**< The graph of active options and states. */
  const unsigned& timeStamp; /**< When was the last activation graph received? */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class StateMachineBehaviorWidget;
};
