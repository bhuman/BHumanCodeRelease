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
  slider->setTickInterval(10);

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

void ThresholdSelector::setEnabled(bool value)
{
  slider->setEnabled(value);
  lineEdit->setEnabled(value);
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

void ColorSelector::updateWidgets()
{
  setEnabled(true);
  setTitle("Color Delimiter");
  updateSlider(parent->colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
}

void ColorSelector::updateColorCalibration(int value)
{
  ThresholdSelector::updateColorCalibration(value, parent->colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
  parent->colorCalibrationView.console.colorCalibrationChanged = true;
}

void BlackWhiteSelector::updateWidgets()
{
  setEnabled(true);
  setTitle("Black-White Delimiter");
  updateSlider(parent->colorCalibrationView.console.colorCalibration.blackWhiteDelimiter);
}

void BlackWhiteSelector::updateColorCalibration(int value)
{
  ThresholdSelector::updateColorCalibration(value, parent->colorCalibrationView.console.colorCalibration.blackWhiteDelimiter);
  parent->colorCalibrationView.console.colorCalibrationChanged = true;
}
