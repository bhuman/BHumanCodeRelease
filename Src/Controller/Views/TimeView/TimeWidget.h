/*
 * File:   TimeWidget.h
 * Author: Arne Böckmann
 *
 * Created on May 21, 2013, 8:15 PM
 */

#pragma once

#include <QObject>
#include <string>
#include <unordered_map>
#include <QWidget>
#include <SimRobot.h>
#include <QString>

struct Row;
class QTableWidget;
class TimeView;
class QLabel;

/**A widget that is used inside a TimeView to display timings*/
class TimeWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT;

public:
  TimeWidget(TimeView& timeView);
  virtual ~TimeWidget();

  virtual QWidget* getWidget();
  virtual void update();

private:
  QLabel* frequency;
  TimeView& timeView;
  QTableWidget* table; /**< The table that displays the timings */
  unsigned lastTimeInfoTimeStamp = 0;
  std::unordered_map<unsigned short, Row*> items;
  unsigned lastUpdate; /**< time of the last update. Used to manage update rate */
  QString filter; /**< the current filter that has been entered */

  /**
   * Hides all rows from the table that fit the filter
   */
  void applyFilter();
  
  // Create Context Menu
  virtual QMenu* createEditMenu() const;

private slots:
  void filterChanged(const QString& newFilter);
  /** Copy contents of the timing data table to the clipboard in csv format */
  void copy();
};
