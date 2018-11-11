/**
 * File:   RangeSelector.h
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * Author: marcel
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Representations/Configuration/FieldColors.h"

#include <qxtspanslider.h>
#include <QLineEdit>
#include <QString>
#include <QGroupBox>
#include <QIntValidator>
#include <QMouseEvent>

class ColorCalibrationWidget;

class RangeSelector : public QGroupBox
{
  Q_OBJECT
public:
  RangeSelector(const QString& name, ColorCalibrationWidget* parent,
                int min, int max);

  virtual void updateWidgets() = 0;
  void setEnabled(bool value);

protected:
  QxtSpanSlider* slider;
  const ColorCalibrationWidget* parent;
  bool allowedSliderOverRun = true;

  void updateColorCalibration(int value, bool isMin, Rangeuc& range)
  {
    if(isMin)
    {
      range.min = allowedSliderOverRun || value < range.max ? static_cast<unsigned char>(value) : range.max;
      updateSlider(range);
    }
    else
    {
      range.max = allowedSliderOverRun || value > range.min ? static_cast<unsigned char>(value) : range.min;
      updateSlider(range);
    }
  }

  /** must be implemented by subclasses to update slider with specific component of color. */
  void updateSlider(const Rangeuc& range)
  {
    slider->setUpperValue(range.max);
    slider->setLowerValue(range.min);
  }

private:
  QLineEdit* lower;
  QLineEdit* upper;

  /** updates the ColorCalibration, usually invoked by the slider/labels. */
  virtual void updateColorCalibration(int value, bool isMin) = 0;

  /** must be implemented by subclasses to update the specific component of a color. */

private slots:
  void sliderLowerChanged(int value);
  void sliderUpperChanged(int value);
  void labelLowerChanged(QString value);
  void labelUpperChanged(QString value);
};

class HueFieldSelector : public RangeSelector
{
public:
  HueFieldSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
    RangeSelector(name, parent, min, max)
  {
    allowedSliderOverRun = false;
  }

  void updateWidgets() override;

private:
  void updateColorCalibration(int value, bool isMin) override;
};
