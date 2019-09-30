/**
 * @file Controller/Views/JointView.h
 * Declaration of class for displaying the requested and measured joint angles.
 * @author Colin Graf
 */

#pragma once

#include <SimRobot.h>
#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QFontMetrics>
#include <QSettings>

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
  SimRobot::Widget* createWidget() override;
  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }
};

class JointWidget : public QWidget
{
  Q_OBJECT

private:
  JointView& jointView;
  unsigned lastUpdateTimestamp = 0; /**< Timestamp of the last painted joint angles. */

  QHeaderView* headerView;

  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField0;
  QRect paintRectField1;
  QRect paintRectField2;
  QRect paintRectField3;
  QRect paintRectField4;
  QRect paintRectField5;

public:
  JointWidget(JointView& jointView, QHeaderView* headerView, QWidget* parent);
  ~JointWidget();

  void update();
  void paintEvent(QPaintEvent* event) override;

public slots:
  void forceUpdate();

private:
  void print(const char* name, const char* value1, const char* value2, const char* value3, const char* value4, const char* value5);
  void newSection();
  QSize sizeHint() const override { return QSize(260, 400); }
};
