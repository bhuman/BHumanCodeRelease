/**
 * File:   ThresholdSelector.h
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * Author: marcel
 *
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Representations/Configuration/FieldColors.h"

#include <QSlider>
#include <QLineEdit>
#include <QString>
#include <QGroupBox>

class ColorCalibrationWidget;

class ThresholdSelector : public QGroupBox
{
  Q_OBJECT

public:
  ThresholdSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max);

  virtual void updateWidgets() = 0;
  void setEnabled(bool value);

protected:
  QSlider* slider;
  const ColorCalibrationWidget* parent;

  void updateColorCalibration(int value, unsigned char& color)
  {
    color = static_cast<unsigned char>(value);
  }

  void updateSlider(unsigned char color)
  {
    slider->setValue(static_cast<int>(color));
  }

private:
  QLineEdit* lineEdit;

  /** updates the ColorCalibration, usually invoked by the slider/labels. */
  virtual void updateColorCalibration(int value) = 0;

private slots:
  void sliderChanged(int value);
  void lineEditChanged(QString value);
};

class ColorSelector : public ThresholdSelector
{
public:
  ColorSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
    ThresholdSelector(name, parent, min, max)
  {}

  void updateWidgets() override;

private:
  void updateColorCalibration(int value) override;
};

class BlackWhiteSelector : public ThresholdSelector
{
public:
  BlackWhiteSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
    ThresholdSelector(name, parent, min, max)
  {}

  void updateWidgets() override;

private:
  void updateColorCalibration(int value) override;
};
