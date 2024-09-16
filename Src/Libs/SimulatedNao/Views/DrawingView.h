/**
 * @file DrawingView.h
 *
 * This file declares a base class for 2D views that display drawings.
 *
 * @author Thomas Röfer
 */

#pragma once

#include <QIcon>
#include <QMouseEvent>
#include <QPainter>
#include <QString>
#include <QWidget>
#include <SimRobot.h>
#include <functional>
#include "SimulatedNao/RobotConsole.h"

/** The view as it is maintained by SimRobot. */
class DrawingView : public SimRobot::Object
{
public:
  /**
   * Initializes a new drawing view.
   * @param fullName The path to this view in the scene graph.
   * @param console The console object.
   * @param name The name of the view.
   * @param threadName The name of the associated thread. If empty, no thread is associated.
   * @param ySign The sign for y coordinates.
   */
  DrawingView(const QString& fullName, RobotConsole& console,
              const std::string& name, const std::string& threadName,
              const char* iconName, float ySign);

  /** @return The path to this view in the scene graph. */
  const QString& getFullName() const override {return fullName;}

  /** @return The icon of this view in the scene graph. */
  const QIcon* getIcon() const override {return &icon;}

protected:
  const QString fullName; /**< The path to this view in the scene graph */
  QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const std::string name; /**< The name of the view. */
  const std::string threadName; /**< The thread that created the images shown in this view. */
  const float ySign; /**< The sign for y coordinates. */

  friend class DrawingWidget;
  friend class ImageWidget;
  friend class FieldWidget;
  friend class SACFieldWidget;
};

/** The widget that is placed in the client area of the dock widget. */
class DrawingWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  /**
   * Constructs the widget and reads layout settings.
   * @param view The view that belongs to this widget.
   * @param origin The origin relative to the center in the range [-0.5..0.5] for both dimensions.
   * @param drawings The drawings shown by this view.
   */
  DrawingWidget(DrawingView& view, const QPointF& origin, const std::list<std::string>& drawings);

  /** Saves the layout settings and deconstructs the widget. */
  ~DrawingWidget() override;

protected:
  /** @return This widget. */
  QWidget* getWidget() override {return this;}

  /** Called to update the view. It is only actually updated if it \c needsRepaint . */
  void update() override;

  /** Called when the view must be repainted. */
  void paintEvent(QPaintEvent*) override;

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

  /**
   * Sets the transform.
   * @param painter The painter the transform is set in.
   */
  void updateTransform(QPainter& painter);

  /**
   * Paint all drawings that are active.
   * @param painter The graphics context to paint to.
   * @param drawings The names of the drawings to paint.
   */
  void paintDrawings(QPainter& painter);

  /** @return Is it necessary to repaint this view? */
  virtual bool needsRepaint() const;

  /**
   * Transforms a point in window coordinate to logical coordinates.
   * @param point The point that is transformed.
   */
  void window2viewport(QPointF& point);

  /**
   * Abstract method to find all drawings of a certain name. If this view is associated
   * with a thread and that thread can provide the drawing, only the drawing from
   * that thread will be returned (or none if the thread currently is not providing the
   * drawing). Derived classes that implement this method will call the next method.
   * @param name The name of the drawing(s) to return.
   * @return The drawings.
   */
  virtual std::vector<std::pair<std::string, const DebugDrawing*>> getDrawings(const std::string& name) const = 0;

  /**
   * Find all drawings of a certain name. If this view is associated with a thread and
   * that thread can provide the drawing, only the drawing from that thread will be
   * returned (or none if the thread currently is not providing the drawing).
   * @param name The name of the drawing(s) to return.
   * @param thread2Drawings A function that returns a map of drawings for a specific thread.
   * @return The drawings.
   */
  std::vector<std::pair<std::string, const DebugDrawing*>> getDrawings(const std::string& name,
    const std::function<const RobotConsole::Drawings&(const RobotConsole::ThreadData&)>& thread2Drawings) const;

  DrawingView& view; /**< The view that belongs to this widget. */
  const QPointF origin; /**< The initial offset . */
  const std::list<std::string>& drawings; /**< The drawings shown by this view. */
  QSize viewSize; /**< The logical size of the view. Must be set by by the derived class. */
  unsigned int lastDrawingsTimestamp = 0; /**< The timestamp of the newest debug drawing that was already drawn. */
  QPainter painter; /**< The graphics context used for painting. */
  QPointF dragStart; /**< The position in window coordinates where dragging started. (-1, -1) if currently not dragging. **/
  QPointF dragStartOffset; /**< The value of \c offset when dragging started. */
  float zoom = 1.f; /**< The zoom ratio. */
  float scale = 1.f; /**< The scale of debug drawings. */
  QPointF offset; /**< The offset of the content relative to the window in logical coordinates. */

private:
  static constexpr Rangef zoomRange{0.1f, 500.f}; /**< The range to which \c zoom is clamped. */
  static constexpr float offsetStepRatio = 0.02f; /**< The step size for moving the content using the cursor key relative to the size of the view. */
};
