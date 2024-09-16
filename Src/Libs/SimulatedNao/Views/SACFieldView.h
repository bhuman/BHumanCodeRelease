/**
 * @file SimulatedNao/Views/SACFieldView.h
 *
 * Declaration of class SACFieldView
 *
 * @author Tatjana Thielke
 */

#pragma once

#include "FieldView.h"

/**
 * @class SACFieldView
 *
 * A class to represent a view displaying debug drawings in field coordinates for the Shared Autonomy Challenge.
 *
 * @author Tatjana Thielke
 */
class SACFieldView : public FieldView
{
  friend class SACControlWidget;

public:
  /**
   * Initializes a new field view.
   * @param fullName The path to this view in the scene graph.
   * @param console The console object.
   * @param name The name of the view.
   * @param threadName The name of the associated thread. If empty, no thread is associated.
   */
  SACFieldView(const QString& fullName, RobotConsole& console,
               const std::string& name, const std::string& threadName) : FieldView(fullName, console, name, threadName)
  {
  }

protected:
  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;
};

class SACFieldWidget : public FieldWidget
{
  Q_OBJECT

  friend class SACControlWidget;

public:
  SACFieldWidget(SACFieldView& view) : FieldWidget(view)
  {
    viewSize = QSize(viewSize.height(), viewSize.width());
  }

private:

  void paint(QPainter& painter) override;

  /**
   * Called if a mouse button was pressed inside the view.
   * @param event Information about the button press.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * Called when the mouse pointer moves inside the view.
   * @param event Information about the mouse pointers position.
   */
  void mouseMoveEvent(QMouseEvent* event) override;

  /**
   * Called when a mouse button is released.
   * @param event Just passed through to base class.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * Called if a mouse button was double-clicked inside the view.
   * @param event Information about the button press.
   */
  void mouseDoubleClickEvent(QMouseEvent* event) override;

  /**
   * Called if the mouse wheel was used.
   * @param event Information about the mouse wheel and the mouse position.
   */
  void wheelEvent(QWheelEvent* event) override;

  /**
   * Called if the key was pressed when the view has focus.
   * @param event Information about the key that was pressed.
   */
  void keyPressEvent(QKeyEvent* event) override;

  /**
   * Called for any event. Use here to handle pinch gestures to change the zoom.
   * @param event Information about the event.
   */
  bool event(QEvent* event) override;
};
