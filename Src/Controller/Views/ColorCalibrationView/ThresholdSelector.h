/**
 * File:   ThresholdSelector.h
 * Author: marcel
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 *
 * Created on October 16, 2013, 12:22 PM
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

  void updateWidgets();
  void setEnabled(bool value);

protected:
  QSlider* slider;

private:
  const ColorCalibrationWidget* parent;
  QLineEdit* lineEdit;

  /** updates the ColorCalibration, usually invoked by the slider/labels. */
  void updateColorCalibration(int value);

  void updateColorCalibration(int value, unsigned char& color)
  {
    color = static_cast<unsigned char>(value);
  }

  void updateSlider(unsigned char color)
  {
    slider->setValue(static_cast<int>(color));
  }

private slots:
  void sliderChanged(int value);
  void lineEditChanged(QString value);
};
