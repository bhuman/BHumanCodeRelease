/*
 * File:   ColorCalibrationView.h
 * Author: Marcel
 *
 * Created on June 25, 2013, 8:13 PM
 */

#pragma once

#include <SimRobot.h>
#include <QMenu>
#include "Controller/RobotConsole.h"
#include "RangeSelector.h"
#include "ThresholdSelector.h"

class ColorCalibrationWidget;

class ColorCalibrationView : public SimRobot::Object
{
public:
  RobotConsole& console;
  ColorCalibrationWidget* widget;

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
  ColorClasses::Color currentColor;
  unsigned timeStamp;

  ColorCalibrationWidget(ColorCalibrationView& colorCalibrationView);
  virtual ~ColorCalibrationWidget();
  virtual QWidget* getWidget();
  virtual void update();
  void updateWidgets(ColorClasses::Color currentColor);
  virtual QMenu* createUserMenu() const;

private slots:
  void saveColorCalibration();
  void colorAct(int color) {updateWidgets((ColorClasses::Color) color);}

private:
  HueSelector* hue;
  SaturationSelector* saturation;
  IntensitySelector* intensity;

  // color class white
  ThresholdSelector* minR;
  ThresholdSelector* minB;
  ThresholdSelector* minRB;
};

