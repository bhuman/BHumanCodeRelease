/*
 * File:   ColorCalibrationView.h
 * @author marcel
 * @author Andreas Stolpmann
 *
 * Created on June 25, 2013, 8:13 PM
 */

#pragma once

#include <SimRobot.h>
#include <QMenu>
#include "Controller/RobotConsole.h"
#include "RangeSelector.h"
#include "ThresholdSelector.h"
#include "History.h"

class ColorCalibrationWidget;

class ColorCalibrationView : public SimRobot::Object
{
public:
  RobotConsole& console;
  ColorCalibrationWidget* widget = nullptr;

  ColorCalibrationView(const QString& fullName, RobotConsole& console);

  SimRobot::Widget* createWidget() override;
  const QString& getFullName() const override;
  const QIcon* getIcon() const override;

private:
  const QString fullName;
  const QIcon icon;
};

class ColorCalibrationWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  ColorCalibrationView& colorCalibrationView;
  unsigned timestamp = 0;

  ColorCalibrationWidget(ColorCalibrationView& colorCalibrationView);
  ~ColorCalibrationWidget();
  QWidget* getWidget() override;
  void update() override;
  void updateWidgets();
  QMenu* createUserMenu() const override;

  void setUndoRedo();
  void expandCurrentColor(const PixelTypes::YUYVPixel& pixel, const bool reduce);

private:
  HueFieldSelector* hueField;

  // color class white
  ThresholdSelector* thresholdColor;
  ThresholdSelector* thresholdBlackWhite;

  QAction* redoAction = nullptr;
  QAction* undoAction = nullptr;

  History<FieldColors> history;

public slots:
  void currentCalibrationChanged();

private slots:
  void undoColorCalibration();
  void redoColorCalibration();
  void saveColorCalibration();

private:
  /**
   *The toolbar of a widget (and therefore the containing actions) is deleted by the
   * SimRobot mainwindow if another view receives focus. If this happens there is no
   * way to know for this widget that the toolbar (and therefor the undo/redo buttons)
   * is deleted. So this work around sets the undo/redo button bointers to nullptr if
   * they are deleted.
   */
  class WorkAroundAction : public QAction
  {
  private:
    QAction** toBeSetToNULL;
  public:
    WorkAroundAction(QAction** toBeSetToNULL, const QIcon& icon, const QString& text, QObject* parent) :
      QAction(icon, text, parent), toBeSetToNULL(toBeSetToNULL)
    {}

    ~WorkAroundAction()
    {
      (*toBeSetToNULL) = nullptr;
    }
  };
};
