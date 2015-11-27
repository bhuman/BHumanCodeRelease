/*
 * File:   RangeSelector.h
 * Author: marcel
 *
 * Created on September 27, 2013, 3:32 PM
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Representations/Configuration/ColorCalibration.h"

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

  void updateWidgets();
  void setEnabled(bool value);

protected:
  QxtSpanSlider* slider;
  const ColorCalibrationWidget* parent;

private:
  QLineEdit* lower;
  QLineEdit* upper;

  /** updates the ColorCalibration, usually invoked by the slider/labels. */
  void updateColorCalibration(int value, bool isMin);

  /** must be implemented by subclasses to update the specific component of a color. */
  virtual void updateColorCalibration(int value, bool isMin,
                                    ColorCalibration::HSIRanges& color) = 0;

  /** must be implemented by subclasses to update slider with specific component of color. */
  virtual void updateSlider(ColorCalibration::HSIRanges& color) = 0;

private slots:
  void sliderLowerChanged(int value);
  void sliderUpperChanged(int value);
  void labelLowerChanged(QString value);
  void labelUpperChanged(QString value);
};


class HueSelector : public RangeSelector
{
private:
  void updateColorCalibration(int value, bool isMin,
                            ColorCalibration::HSIRanges& color)
  {
    if(isMin)
      color.hue.min = value;
    else
      color.hue.max = value;
  }

  void updateSlider(ColorCalibration::HSIRanges& color)
  {
    slider->setUpperValue(color.hue.max);
    slider->setLowerValue(color.hue.min);
  }

public:
  HueSelector(const QString& name, ColorCalibrationWidget* parent,
              int min, int max)
  : RangeSelector(name, parent, min, max) {}
};

class SaturationSelector : public RangeSelector
{
private:

  void updateColorCalibration(int value, bool isMin,
                              ColorCalibration::HSIRanges& color)
  {
    if(isMin)
      color.saturation.min = value;
    else
      color.saturation.max = value;
  }

  void updateSlider(ColorCalibration::HSIRanges& color)
  {
    slider->setUpperValue(color.saturation.max);
    slider->setLowerValue(color.saturation.min);
  }

public:
  SaturationSelector(const QString& name, ColorCalibrationWidget* parent,
                     int min, int max)
  : RangeSelector(name, parent, min, max) {}
};

class IntensitySelector : public RangeSelector
{
private:
  void updateColorCalibration(int value, bool isMin,
                              ColorCalibration::HSIRanges& color)
  {
    if(isMin)
      color.intensity.min = value;
    else
      color.intensity.max = value;
  }

  void updateSlider(ColorCalibration::HSIRanges& color)
  {
    slider->setUpperValue(color.intensity.max);
    slider->setLowerValue(color.intensity.min);
  }

public:
  IntensitySelector(const QString& name, ColorCalibrationWidget* parent,
                int min, int max)
  : RangeSelector(name, parent, min, max) {}
};
