/**
 * @file Controller/Views/FieldView.h
 *
 * Declaration of class FieldView
 *
 * @author Thomas Röfer
 */

#pragma once

#include <QString>
#include <QIcon>
#include <QPainter>

#include <SimRobot.h>
#include "Controller/RobotConsole.h"
#include "Representations/Configuration/FieldDimensions.h"

class RobotConsole;

/**
 * @class FieldView
 *
 * A class to represent a view displaying debug drawings in field coordinates.
 *
 * @author Thomas Röfer
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

class FieldWidget : public WIDGET2D, public SimRobot::Widget
{
public:
  FieldWidget(FieldView& fieldView);
  ~FieldWidget();

private:
  FieldView& fieldView;
  FieldDimensions fieldDimensions; /**< The field dimensions. */
  unsigned lastDrawingsTimestamp = 0;
  QPainter painter;
  QPointF dragStart;
  QPointF dragStartOffset;
  QPointF mousePos;
  float zoom = 1.f;
  float scale = 1.f;
  QPointF offset;

  void paintEvent(QPaintEvent* event) override;
  void paint(QPainter& painter) override;
  void paintDrawings(QPainter& painter);
  bool needsRepaint() const;
  void window2viewport(QPointF& point);
  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  bool event(QEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  std::vector<std::pair<std::string, const DebugDrawing*>> getDrawings(const std::string& name) const;

  QSize sizeHint() const override { return QSize(int(fieldDimensions.xPosOpponentFieldBorder * 0.2f), int(fieldDimensions.yPosLeftFieldBorder * 0.2f)); }

  QWidget* getWidget() override { return this; }

  void update() override
  {
    if(needsRepaint())
      QWidget::update();
  }

  QMenu* createUserMenu() const override;

  friend class FieldView;
};
