/**
 * @file SimulatedNao/Views/FieldView.h
 *
 * Declaration of class FieldView
 *
 * @author Thomas Röfer
 */

#pragma once

#include "DrawingView.h"

/**
 * @class FieldView
 *
 * A class to represent a view displaying debug drawings in field coordinates.
 *
 * @author Thomas Röfer
 */
class FieldView : public DrawingView
{
public:
  /**
   * Initializes a new field view.
   * @param fullName The path to this view in the scene graph.
   * @param console The console object.
   * @param name The name of the view.
   * @param threadName The name of the associated thread. If empty, no thread is associated.
   */
  FieldView(const QString& fullName, RobotConsole& console,
            const std::string& name, const std::string& threadName);

protected:
  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;
};

class FieldWidget : public DrawingWidget
{
  Q_OBJECT

public:
  FieldWidget(FieldView& view);

private:
  void paint(QPainter& painter) override;
  std::vector<std::pair<std::string, const DebugDrawing*>> getDrawings(const std::string& name) const override;
  QSize sizeHint() const override { return viewSize / 20; }
  QMenu* createUserMenu() const override;
};
