/**
 * @file Controller/Views/CABSLBehaviorView.h
 * Declaration of class CABSLBehaviorView
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include <SimRobot.h>

class RobotConsole;
struct ActivationGraph;

/**
 * @class CABSLBehaviorView
 * A class to represent a view with information about a CABSL behavior.
 */
class CABSLBehaviorView : public SimRobot::Object
{
public:
  /**
   * @param fullName The full name of this view.
   * @param console The console object.
   * @param activationGraph The graph of active options and states to be visualized.
   * @param timestamp When was the last activation graph received?
   */
  CABSLBehaviorView(const QString& fullName, RobotConsole& console, const ActivationGraph& activationGraph, const unsigned& timestamp);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const ActivationGraph& activationGraph; /**< The graph of active options and states. */
  const unsigned& timestamp; /**< When was the last activation graph received? */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class CABSLBehaviorWidget;
};
