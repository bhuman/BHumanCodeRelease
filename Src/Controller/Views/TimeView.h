/**
 * @file Controller/Views/TimeView.h
 *
 * Declaration of class TimeView
 *
 * @author Colin Graf
 * @author Arne Böckmann
 */

#pragma once

#include <SimRobot.h>
#include <unordered_map>
#include <string>
#include <QIcon>
#include <QObject>
#include <QWidget>
#include <QString>

class NumberTableWidgetItem;
class RobotConsole;
struct Row;
class TimeInfo;
class TimeWidget;
class QLabel;
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
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class TimeWidget;//TimeWidget needs access to console
};

/**A widget that is used inside a TimeView to display timings*/
class TimeWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

public:
  TimeWidget(TimeView& timeView);
  ~TimeWidget();

  QWidget* getWidget() override;
  void update() override;

private:
  QLabel* frequency;
  TimeView& timeView;
  QTableWidget* table; /**< The table that displays the timings */
  unsigned lastTimeInfoTimestamp = 0;
  std::unordered_map<unsigned short, Row*> items;
  unsigned lastUpdate; /**< time of the last update. Used to manage update rate */
  QString filter; /**< the current filter that has been entered */

  /**
   * Hides all rows from the table that fit the filter
   */
  void applyFilter();

  // Create Context Menu
  QMenu* createEditMenu() const override;

private slots:
  void filterChanged(const QString& newFilter);
  /** Copy contents of the timing data table to the clipboard in csv format */
  void copy();
};
