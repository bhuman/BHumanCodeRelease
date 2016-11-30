/*
 * File:   ThresholdSelector.cpp
 * Author: marcel
 *
 * Created on October 16, 2013, 12:22 PM
 */

#include "ThresholdSelector.h"
#include "ColorCalibrationView.h"

#include <QHBoxLayout>

ThresholdSelector::ThresholdSelector(const QString& name, ColorCalibrationWidget* parent, int min, int max) :
  QGroupBox(name, parent), parent(parent)
{
  slider = new QSlider(Qt::Orientation::Horizontal, this);
  slider->setMinimum(min);
  slider->setMaximum(max);
  slider->setTickPosition(QSlider::TicksBothSides);

  lineEdit = new QLineEdit(QString::number(min), this);
  lineEdit->setFixedWidth(40);
  lineEdit->setValidator(new QIntValidator(min, max, lineEdit));

  connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged(int)));
  connect(lineEdit, SIGNAL(textEdited(QString)), this, SLOT(lineEditChanged(QString)));

  connect(slider, SIGNAL(sliderReleased()), parent, SLOT(currentCalibrationChanged()));
  connect(lineEdit, SIGNAL(editingFinished()), parent, SLOT(currentCalibrationChanged()));

  QHBoxLayout* layout = new QHBoxLayout(this);
  setLayout(layout);
  layout->addWidget(lineEdit);
  layout->addWidget(slider);
}

void ThresholdSelector::updateWidgets()
{
  if(parent->currentColor == FieldColors::none)
  {
    setEnabled(true);
    setTitle("Max Black/White Saturation");
    updateSlider(parent->colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
  }
  else if(parent->currentColor == FieldColors::white)
  {
    setEnabled(true);
    setTitle("Y-Threshold White");
    updateSlider(parent->colorCalibrationView.console.colorCalibration.minYWhite);
  }
  else if(parent->currentColor == FieldColors::black)
  {
    setEnabled(true);
    setTitle("Y-Threshold Black");
    updateSlider(parent->colorCalibrationView.console.colorCalibration.maxYBlack);
  }
  else
    setEnabled(false);
}

void ThresholdSelector::setEnabled(bool value)
{
  slider->setEnabled(value);
  lineEdit->setEnabled(value);
}

void ThresholdSelector::updateColorCalibration(int value)
{
  if(parent->currentColor == FieldColors::none)
  {
    updateColorCalibration(value, parent->colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
    parent->colorCalibrationView.console.colorCalibrationChanged = true;
  }
  if(parent->currentColor == FieldColors::white)
  {
    updateColorCalibration(value, parent->colorCalibrationView.console.colorCalibration.minYWhite);
    parent->colorCalibrationView.console.colorCalibrationChanged = true;
  }
  else if(parent->currentColor == FieldColors::black)
  {
    updateColorCalibration(value, parent->colorCalibrationView.console.colorCalibration.maxYBlack);
    parent->colorCalibrationView.console.colorCalibrationChanged = true;
  }
}

void ThresholdSelector::sliderChanged(int value)
{
  lineEdit->setText(QString::number(value));
  updateColorCalibration(value);
}

void ThresholdSelector::lineEditChanged(QString value)
{
  slider->setValue(value.toInt());
}
