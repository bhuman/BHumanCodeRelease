/*
 * File:   ThresholdSelector.h
 * Author: marcel
 *
 * Created on October 16, 2013, 12:22 PM
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Representations/Configuration/ColorCalibration.h"

#include <QSlider>
#include <QLineEdit>
#include <QString>
#include <QGroupBox>

class ColorCalibrationWidget;

class ThresholdSelector : public QGroupBox
{
  Q_OBJECT

public:
  ThresholdSelector(const QString& name, ColorCalibrationWidget* parent,
                    int min, int max);

  void updateWidgets();
  void setEnabled(bool value);

protected:
  QSlider* slider;

private:
  const ColorCalibrationWidget* parent;
  QLineEdit* lineEdit;

  /** updates the ColorCalibration, usually invoked by the slider/labels. */
  void updateColorCalibration(int value);

  /** must be implemented by subclasses to update the specific component of a color. */
  virtual void updateColorCalibration(int value, ColorCalibration::WhiteThresholds& color) = 0;

  /** must be implemented by subclasses to update widget with specific component of color. */
  virtual void updateSlider(ColorCalibration::WhiteThresholds& color) = 0;

private slots:
  void sliderChanged(int value);
  void lineEditChanged(QString value);
};

class MinRSelector : public ThresholdSelector
{
private:
  void updateColorCalibration(int value, ColorCalibration::WhiteThresholds& color)
  {
    color.minR = slider->value();
  }

  void updateSlider(ColorCalibration::WhiteThresholds& color)
  {
    slider->setValue(color.minR);
  }

public:
  MinRSelector(const QString& name, ColorCalibrationWidget* parent,
               int min, int max)
  : ThresholdSelector(name, parent, min, max) {}
};

class MinBSelector : public ThresholdSelector
{
private:
  void updateColorCalibration(int value, ColorCalibration::WhiteThresholds& color)
  {
    color.minB = slider->value();
  }

  void updateSlider(ColorCalibration::WhiteThresholds& color)
  {
    slider->setValue(color.minB);
  }

public:
  MinBSelector(const QString& name, ColorCalibrationWidget* parent,
               int min, int max)
  : ThresholdSelector(name, parent, min, max) {}
};

class MinRBSelector : public ThresholdSelector
{
private:
  void updateColorCalibration(int value, ColorCalibration::WhiteThresholds& color)
  {
    color.minRB = slider->value();
  }

  void updateSlider(ColorCalibration::WhiteThresholds& color)
  {
    slider->setValue(color.minRB);
  }

public:
  MinRBSelector(const QString& name, ColorCalibrationWidget* parent,
                int min, int max)
  : ThresholdSelector(name, parent, min, max) {}
};
