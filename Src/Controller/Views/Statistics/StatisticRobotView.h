/**
 * @file Controller/Views/Evaluation/StatisticRobotView.h
 *
 * Declaration of class StatisticRobotView
 *
 * @author <a href="mailto:jan_fie@uni-bremen.de">Jan Fiedler</a>
 */

#pragma once

#include "SimRobot.h"
#include <QHeaderView>
#include <QApplication>
#include <QPainter>
#include <QSettings>
#include <vector>
#include "Controller/Statistics.h"

class StatisticRobotWidget;

/**
 * @class StatisticRobotView
 * A class to represent a view with information about the Robot, extracted from Logs.
 */
class StatisticRobotView : public SimRobot::Object
{
private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  const std::vector<Statistic::StatisticRobot>& robots;
  const size_t robotIndex;
  const unsigned& timeStamp;

  friend class StatisticRobotWidget;

public:
  /**
   * @param fullName The path to this view in the scene graph
   * @param robots The list of all robots.
   * @param robotIndex The index of the robot to be displayed.
   * @param timeStamp A reference to the last modification time. Can be 0, if never changed.
   */
  StatisticRobotView(const QString& fullName, const std::vector<Statistic::StatisticRobot>& robots, const size_t robotIndex, const unsigned& timeStamp = 0);

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

class StatisticRobotWidget : public QWidget
{
  Q_OBJECT

private:
  StatisticRobotView& statisticRobotView;
  QHeaderView* headerView;

  QPainter painter;
  int lineSpacing;
  int textOffset;

  QFont font;
  QBrush altBrush;
  QPen fontPen;
  QPen noPen;
  bool fillBackground;

  QRect paintRect;
  QRect paintRectField;
  QRect paintRectField0;
  QRect paintRectField1;
  unsigned lastUpdateTimeStamp = 0;

public:
  StatisticRobotWidget(StatisticRobotView& statisticRobotView, QHeaderView* headerView, QWidget* parent);
  ~StatisticRobotWidget();

  void update();
  void paintEvent(QPaintEvent* event) override;

public slots:
  void forceUpdate();

private:
  void printJoints();
  void print(const QString& name, const QString& value);
  void printLine(const QString& value, const bool changeColor = true);
  void printList(const std::vector<unsigned>& list, const size_t lineElements = 5);
  void newSection();
  QSize sizeHint() const override { return QSize(250, 400); }
  std::string getNamebyNumber(const std::vector<Statistic::StatisticRobot>& robots, int number);
};
