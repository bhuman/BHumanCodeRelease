/**
 * File:   RangeSelector.cpp
 * Author: marcel
 *
 * Created on September 27, 2013, 3:32 PM
 */

#include "RangeSelector.h"
#include "ColorCalibrationView.h"
#include "Platform/BHAssert.h"

#include <QHBoxLayout>
#include <cmath>

RangeSelector::RangeSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
  QGroupBox(name, parent), parent(parent)
{
  QHBoxLayout* layout = new QHBoxLayout(this);
  setLayout(layout);

  slider = new QxtSpanSlider(Qt::Orientation::Horizontal, this);
  slider->setTickPosition(QSlider::TicksBothSides);
  slider->setTickInterval(10);
  slider->setHandleMovementMode(QxtSpanSlider::HandleMovementMode::FreeMovement);
  slider->setMinimum(min);
  slider->setMaximum(max);

  lower = new QLineEdit(QString::number(min), this);
  lower->setFixedWidth(40);
  lower->setValidator(new QIntValidator(min, max, lower));
  upper = new QLineEdit(QString::number(min), this);
  upper->setFixedWidth(40);
  upper->setValidator(new QIntValidator(min, max, upper));

  connect(slider, SIGNAL(lowerValueChanged(int)), this, SLOT(sliderLowerChanged(int)));
  connect(slider, SIGNAL(upperValueChanged(int)), this, SLOT(sliderUpperChanged(int)));
  connect(lower, SIGNAL(textEdited(QString)), this, SLOT(labelLowerChanged(QString)));
  connect(upper, SIGNAL(textEdited(QString)), this, SLOT(labelUpperChanged(QString)));

  connect(slider, SIGNAL(sliderReleased()), parent, SLOT(currentCalibrationChanged()));
  connect(upper, SIGNAL(editingFinished()), parent, SLOT(currentCalibrationChanged()));
  connect(lower, SIGNAL(editingFinished()), parent, SLOT(currentCalibrationChanged()));

  layout->addWidget(lower);
  layout->addWidget(slider);
  layout->addWidget(upper);
  setEnabled(false);
}

void RangeSelector::setEnabled(bool value)
{
  slider->setEnabled(value);
  lower->setEnabled(value);
  upper->setEnabled(value);
}

void RangeSelector::sliderLowerChanged(int value)
{
  lower->setText(QString::number(value));
  updateColorCalibration(value, true);
}

void RangeSelector::sliderUpperChanged(int value)
{
  upper->setText(QString::number(value));
  updateColorCalibration(value, false);
}

void RangeSelector::labelLowerChanged(QString value)
{
  slider->setLowerValue(value.toInt());
}

void RangeSelector::labelUpperChanged(QString value)
{
  slider->setUpperValue(value.toInt());
}

void HueFieldSelector::updateWidgets()
{
  setEnabled(true);

  updateSlider(parent->colorCalibrationView.console.colorCalibration.fieldHue);
}

void HueFieldSelector::updateColorCalibration(int value, bool isMin)
{
  RangeSelector::updateColorCalibration(value, isMin, parent->colorCalibrationView.console.colorCalibration.fieldHue);

  parent->colorCalibrationView.console.colorCalibrationChanged = true;
}
