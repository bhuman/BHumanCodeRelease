/* 
 * File:   RangeSelector.h
 * Author: marcel
 *
 * Created on September 27, 2013, 3:32 PM
 */

#pragma once

#include <Tools/Math/Common.h>
#include <Representations/Perception/ColorReference.h>

#include <qxtspanslider.h>
#include <QLineEdit>
#include <QString>
#include <QGroupBox>
#include <QDoubleValidator>

class ColorCalibrationWidget;

class RangeSelector : public QGroupBox 
{
  Q_OBJECT
  
public:
  RangeSelector(const QString& name, ColorCalibrationWidget* parent,
                const float min, const float max, 
                const unsigned int decimals);
  
  ~RangeSelector() {};
  
  void updateWidgets();
  void setEnabled(bool value);
protected:
  const ColorCalibrationWidget* parent;
  const unsigned int resolution;
  
  /* the calibration widgets. */
  QxtSpanSlider* slider;
  QLineEdit* lower;
  QLineEdit* upper;
private:
  float minValue;
  float maxValue;
  
  /* if true, the ColorReference will not be updated while slider/labels are changing. */
  bool ignoreUpdateColorReference;
  
  /* maps label <--> slider and vice versa. */
  float toLabelValue(const int value);
  int toSliderValue(const float value);
  
  /* updates the ColorReference, usually invoked by the slider/labels. */
  void updateColorReference(const float min, const float max);
  
  /* must be implemented by subclasses to update the specific component of a color. */
  virtual void updateColorReference(const float min, const float max, 
                                    ColorReference::HSVColorDefinition& color) = 0;
  /* must be implemented by subclasses to update widget with specific component of color. */
  /* HACK: for some reason overloading the function #updateWidgets doesn't compile. */
  virtual void updateWidgetsPrivate(ColorReference::HSVColorDefinition& color) = 0;
  
  
  class Validator : public QDoubleValidator
  {
  public:
    Validator(float bottom, float top, int decimals, QObject* parent)
    : QDoubleValidator(bottom, top, decimals, parent) 
    {
      setNotation(QDoubleValidator::Notation::StandardNotation);
    }
    
    QValidator::State validate(QString& input, int& pos) const
    {
      if (input.contains(","))
        return QDoubleValidator::State::Invalid;
      else
        return QDoubleValidator::validate(input, pos);
    }
  };
  
  class NumberInput : public QLineEdit
  {
  public:
    NumberInput(int init, QWidget* parent, float min, float max, int decimals)
    : QLineEdit(QString::number(init), parent)
    {
      setFixedWidth(40);
      setValidator(new Validator(min, max, decimals, this));
    }
  };

private slots:
  void sliderLowerChanged(int value);
  void sliderUpperChanged(int value);
  void labelLowerChanged(QString value);
  void labelUpperChanged(QString value);
};

class HueSelector : public RangeSelector
{
public:
  HueSelector(const QString& name, ColorCalibrationWidget* parent,
              const float min, const float max,
              const unsigned int decimals)
  : RangeSelector(name, parent, min, max, decimals) {}
  
  void updateColorReference(const float min, const float max, 
                            ColorReference::HSVColorDefinition& color)
  {
    color.hue.min = min;
    color.hue.max = max;
  }
  
  void updateWidgetsPrivate(ColorReference::HSVColorDefinition& color)
  {
    slider->setUpperValue((int) (resolution * color.hue.max));
    slider->setLowerValue((int) (resolution * color.hue.min));
  }
};

class SaturationSelector : public RangeSelector
{
public:
  SaturationSelector(const QString& name, ColorCalibrationWidget* parent,
                     const float min, const float max, const unsigned int decimals)
  : RangeSelector(name, parent, min, max, decimals) {}
  
  void updateColorReference(const float min, const float max, 
                            ColorReference::HSVColorDefinition& color)
  {
    color.saturation.min = min;
    color.saturation.max = max;
  }
  
  void updateWidgetsPrivate(ColorReference::HSVColorDefinition& color)
  {
    slider->setUpperValue((int) (resolution * color.saturation.max));
    slider->setLowerValue((int) (resolution * color.saturation.min));
  }
};

class ValueSelector : public RangeSelector
{
public:
  ValueSelector(const QString& name, ColorCalibrationWidget* parent,
                const float min, const float max, const unsigned int decimals)
  : RangeSelector(name, parent, min, max, decimals) {}
  
  void updateColorReference(const float min, const float max, 
                            ColorReference::HSVColorDefinition& color)
  {
    color.value.min = min;
    color.value.max = max;
  }
  
  void updateWidgetsPrivate(ColorReference::HSVColorDefinition& color)
  {
    slider->setUpperValue((int) (resolution * color.value.max));
    slider->setLowerValue((int) (resolution * color.value.min));
  }
};