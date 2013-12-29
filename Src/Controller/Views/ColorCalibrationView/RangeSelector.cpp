/* 
 * File:   RangeSelector.cpp
 * Author: marcel
 * 
 * Created on September 27, 2013, 3:32 PM
 */

#include "RangeSelector.h"
#include "ColorCalibrationView.h"
#include <Platform/BHAssert.h>

#include <QHBoxLayout>
#include <cmath>

RangeSelector::RangeSelector(const QString& name, ColorCalibrationWidget* parent, 
                             const float min, const float max, 
                             const unsigned int decimals)
: QGroupBox(name, parent), parent(parent), resolution(static_cast<unsigned int>(pow(10, decimals)))
{
  minValue = min * resolution;
  maxValue = max * resolution;
  ignoreUpdateColorReference = false;
  
  VERIFY(maxValue >= minValue);
  
  QHBoxLayout* layout = new QHBoxLayout(this);
  this->setLayout(layout);

  slider = new QxtSpanSlider(Qt::Orientation::Horizontal, this);
  slider->setHandleMovementMode(QxtSpanSlider::HandleMovementMode::FreeMovement);
  slider->setMinimum((int) minValue);
  slider->setMaximum((int) maxValue);
  
  lower = new NumberInput(slider->lowerValue(), this, min, max, 2);
  upper = new NumberInput(slider->upperValue(), this, min, max, 2);
  
  connect(slider, SIGNAL(lowerValueChanged(int)), this, SLOT(sliderLowerChanged(int)));
  connect(slider, SIGNAL(upperValueChanged(int)), this, SLOT(sliderUpperChanged(int)));
  connect(lower, SIGNAL(textEdited(QString)), this, SLOT(labelLowerChanged(QString)));
  connect(upper, SIGNAL(textEdited(QString)), this, SLOT(labelUpperChanged(QString)));

  layout->addWidget(lower);
  layout->addWidget(slider);
  layout->addWidget(upper);
  setEnabled(false);
}

void RangeSelector::updateWidgets()
{
  ignoreUpdateColorReference = true;
  ColorReference* cr = parent->colorReference();
  switch(parent->currentColor)
  {
    case ColorReference::green:
      setEnabled(true);
      updateWidgetsPrivate(cr->thresholdGreen);
      break;
    case ColorReference::yellow:
      setEnabled(true);
      updateWidgetsPrivate(cr->thresholdYellow);
      break;
    case ColorReference::orange:
      setEnabled(true);
      updateWidgetsPrivate(cr->thresholdOrange);
      break;
    case ColorReference::red:
      setEnabled(true);
      updateWidgetsPrivate(cr->thresholdRed);
      break;
    case ColorReference::blue:
      setEnabled(true);
      updateWidgetsPrivate(cr->thresholdBlue);
      break;
    default:
      setEnabled(false);
  }
  ignoreUpdateColorReference = false;
}

void RangeSelector::setEnabled(bool value)
{
  slider->setEnabled(value);
  lower->setEnabled(value);
  upper->setEnabled(value);
}


/* ---------------- Private ---------------- */
float RangeSelector::toLabelValue(const int value)
{
  return (float)value / resolution;
}

int RangeSelector::toSliderValue(const float value)
{
  return (int) (value * resolution);
}

void RangeSelector::updateColorReference(const float min, const float max)
{
  if(ignoreUpdateColorReference)
  {
    return;
  }
  
  ColorReference* cr = parent->colorReference();
  switch(parent->currentColor)
  {
    case ColorReference::green:
      updateColorReference(min, max, cr->thresholdGreen);
      break;
    case ColorReference::yellow:
      updateColorReference(min, max, cr->thresholdYellow);
      break;
    case ColorReference::orange:
      updateColorReference(min, max, cr->thresholdOrange);
      break;
    case ColorReference::red:
      updateColorReference(min, max, cr->thresholdRed);
      break;
    case ColorReference::blue:
      updateColorReference(min, max, cr->thresholdBlue);
      break;
    default:
      return;
  }
  cr->changed = true;
}


/* ---------------- Slots ---------------- */
void RangeSelector::sliderLowerChanged(int value)
{
  const float labelValue = toLabelValue(value);
  lower->setText(QString::number(labelValue));
  updateColorReference(labelValue, upper->text().toFloat());
}

void RangeSelector::sliderUpperChanged(int value) 
{
  const float labelValue = toLabelValue(value);
  upper->setText(QString::number(labelValue));
  updateColorReference(lower->text().toFloat(), labelValue);
}

void RangeSelector::labelLowerChanged(QString value)
{
  slider->setLowerValue(toSliderValue(value.toFloat()));
}

void RangeSelector::labelUpperChanged(QString value)
{
  slider->setUpperValue(toSliderValue(value.toFloat()));
}