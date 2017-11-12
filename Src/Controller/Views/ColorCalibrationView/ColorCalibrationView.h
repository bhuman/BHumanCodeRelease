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
#include "Representations/Infrastructure/Image.h"

class ColorCalibrationWidget;

class ColorCalibrationView : public SimRobot::Object
{
public:
  RobotConsole& console;
  ColorCalibrationWidget* widget = nullptr;

  ColorCalibrationView(const QString& fullName, RobotConsole& console);

  virtual SimRobot::Widget* createWidget();
  virtual const QString& getFullName() const;
  virtual const QIcon* getIcon() const;

private:
  const QString fullName;
  const QIcon icon;
};

class ColorCalibrationWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT

public:
  ColorCalibrationView& colorCalibrationView;
  FieldColors::Color currentColor = FieldColors::none;
  bool expandColorMode = false;
  unsigned timeStamp = 0;

  ColorCalibrationWidget(ColorCalibrationView& colorCalibrationView);
  virtual ~ColorCalibrationWidget();
  virtual QWidget* getWidget();
  virtual void update();
  void updateWidgets(FieldColors::Color currentColor);
  virtual QMenu* createUserMenu() const;

  void setUndoRedo();
  void expandCurrentColor(const PixelTypes::YUYVPixel& pixel, const bool reduce);

private:
  HueFieldSelector* hueField;
  HueSelector* hue;

  // color class white
  ThresholdSelector* thresholdColor;
  ThresholdSelector* thresholdBlackWhite;

  QAction* redoAction = nullptr;
  QAction* undoAction = nullptr;

  History<Rangeuc> historyColors[FieldColors::numOfColors - FieldColors::numOfNonColors];
  History<FieldColors::BasicParameters> historyBasic;

  bool expandColor(const unsigned char value, int& min, int& max, const bool noWrapAround);
  bool reduceColor(const unsigned char value, int& min, int& max, const bool boolChangeOnlyMin);
  int calcColorValueDistance(const int a, const int b) const;

public slots:
  void currentCalibrationChanged();
  void undoColorCalibration();
  void redoColorCalibration();

private slots:
  void saveColorCalibration();
  void colorAct(int color);
  void expandColorAct();

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
