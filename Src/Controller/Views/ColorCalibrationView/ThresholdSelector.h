/* 
 * File:   ThresholdSelector.h
 * Author: marcel
 *
 * Created on October 16, 2013, 12:22 PM
 */

#pragma once

#include <Tools/Math/Common.h>
#include <Representations/Perception/ColorReference.h>

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
                    const int min, const int max);
  
  virtual ~ThresholdSelector() {}
  
  void updateWidgets();
  void setEnabled(bool value);
protected:
  const ColorCalibrationWidget* parent;
  
  /* the calibration widgets. */
  QSlider* slider;
  QLineEdit* lineEdit;
private:
  /* if true, the ColorReference will not be updated while slider/labels are changing. */
  bool ignoreUpdateColorReference;
  
  void updateColorReference(const int value);
  
  /* must be implemented by subclasses to update the specific component of a color. */
  virtual void updateColorReference(const int value, ColorReference::ColorThreshold<>& color) = 0;
  /* must be implemented by subclasses to update widget with specific component of color. */
  /* HACK: for some reason overloading the function #updateWidgets doesn't compile. */
  virtual void updateWidgetsPrivate(ColorReference::ColorThreshold<>& color) = 0;
  
private slots:
  void sliderChanged(int value);
  void lineEditChanged(QString value);
};

class MinRSelector : public ThresholdSelector
{
public:
  MinRSelector(const QString& name, ColorCalibrationWidget* parent, 
               const int min, const int max)
  : ThresholdSelector(name, parent, min, max) {}
  
  void updateColorReference(const int value, ColorReference::ColorThreshold<>& color)
  {
    color.first = slider->value();
  }
  
  void updateWidgetsPrivate(ColorReference::ColorThreshold<>& color) 
  {
    slider->setValue(color.first);
  }
};

class MinBSelector : public ThresholdSelector
{
public:
  MinBSelector(const QString& name, ColorCalibrationWidget* parent, 
               const int min, const int max)
  : ThresholdSelector(name, parent, min, max) {}
  
  void updateColorReference(const int value, ColorReference::ColorThreshold<>& color)
  {
    color.second = slider->value();
  }
  
  void updateWidgetsPrivate(ColorReference::ColorThreshold<>& color) 
  {
    slider->setValue(color.second);
  }
};

class MinRBSelector : public ThresholdSelector
{
public:
  MinRBSelector(const QString& name, ColorCalibrationWidget* parent, 
                const int min, const int max)
  : ThresholdSelector(name, parent, min, max) {}
  
  void updateColorReference(const int value, ColorReference::ColorThreshold<>& color)
  {
    color.third = slider->value();
  }
  
  void updateWidgetsPrivate(ColorReference::ColorThreshold<>& color) 
  {
    slider->setValue(color.third);
  }
};

