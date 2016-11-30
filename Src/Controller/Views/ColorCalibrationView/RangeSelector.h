/**
 * File:   RangeSelector.h
 * Author: marcel
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
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
  virtual void updateColorCalibration(int value, bool isMin, FieldColors::ColorRange& color) = 0;

  /** must be implemented by subclasses to update slider with specific component of color. */
  virtual void updateSlider(const FieldColors::ColorRange& color) = 0;

private slots:
  void sliderLowerChanged(int value);
  void sliderUpperChanged(int value);
  void labelLowerChanged(QString value);
  void labelUpperChanged(QString value);
};

class SSelector : public RangeSelector
{
public:
  SSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
    RangeSelector(name, parent, min, max)
  {}

private:
  void updateColorCalibration(int value, bool isMin, FieldColors::ColorRange& color)
  {
    if(isMin)
    {
      color.s.min = value < color.s.max ? static_cast<unsigned char>(value) : color.s.max;
      updateSlider(color);
    }
    else
    {
      color.s.max = value > color.s.min ? static_cast<unsigned char>(value) : color.s.min;
      updateSlider(color);
    }
  }

  void updateSlider(const FieldColors::ColorRange& color)
  {
    slider->setUpperValue(color.s.max);
    slider->setLowerValue(color.s.min);
  }
};

class HSelector : public RangeSelector
{
public:
  HSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
    RangeSelector(name, parent, min, max)
  {}

private:
  void updateColorCalibration(int value, bool isMin, FieldColors::ColorRange& color)
  {
    if(isMin)
    {
      color.h.min = value < color.h.max ? static_cast<unsigned char>(value) : color.h.max;
      updateSlider(color);
    }
    else
    {
      color.h.max = value > color.h.min ? static_cast<unsigned char>(value) : color.h.min;
      updateSlider(color);
    }
  }

  void updateSlider(const FieldColors::ColorRange& color)
  {
    slider->setUpperValue(color.h.max);
    slider->setLowerValue(color.h.min);
  }
};

class YSelector : public RangeSelector
{
public:
  YSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
    RangeSelector(name, parent, min, max)
  {}

private:
  void updateColorCalibration(int value, bool isMin, FieldColors::ColorRange& color)
  {
    if(isMin)
    {
      color.y.min = value < color.y.max ? static_cast<unsigned char>(value) : color.y.max;
      updateSlider(color);
    }
    else
    {
      color.y.max = value > color.y.min ? static_cast<unsigned char>(value) : color.y.min;
      updateSlider(color);
    }
  }

  void updateSlider(const FieldColors::ColorRange& color)
  {
    slider->setUpperValue(color.y.max);
    slider->setLowerValue(color.y.min);
  }
};
