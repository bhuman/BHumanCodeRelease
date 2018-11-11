/**
 * @file Controller/Views/FieldView.h
 *
 * Declaration of class FieldView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <QString>
#include <QIcon>
#include <SimRobot.h>

class RobotConsole;

/**
 * @class FieldView
 *
 * A class to represent a view displaying debug drawings in field coordinates.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */
class FieldView : public SimRobot::Object
{
public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param console The console object.
   * @param name The name of the view.
   */
  FieldView(const QString& fullName, RobotConsole& console, const std::string& name);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const std::string name; /**< The name of the view. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class FieldWidget;
};
