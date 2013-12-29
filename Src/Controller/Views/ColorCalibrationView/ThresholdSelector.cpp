/* 
 * File:   ThresholdSelector.cpp
 * Author: marcel
 * 
 * Created on October 16, 2013, 12:22 PM
 */

#include "ThresholdSelector.h"
#include "ColorCalibrationView.h"

#include <QHBoxLayout>

ThresholdSelector::ThresholdSelector(const QString& name, ColorCalibrationWidget* parent,
                                     const int min, const int max)
: QGroupBox(name, parent), parent(parent)
{
  slider = new QSlider(Qt::Orientation::Horizontal, this);
  slider->setMinimum(min);
  slider->setMaximum(max);
  
  lineEdit = new QLineEdit(QString::number(min), this);
  lineEdit->setFixedWidth(40);
  lineEdit->setValidator(new QIntValidator(min, max, lineEdit));
  
  connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged(int)));
  connect(lineEdit, SIGNAL(textEdited(QString)), this, SLOT(lineEditChanged(QString)));
  
  QHBoxLayout* layout = new QHBoxLayout(this);
  this->setLayout(layout);
  layout->addWidget(lineEdit);
  layout->addWidget(slider);
}

void ThresholdSelector::updateWidgets()
{
  ignoreUpdateColorReference = true;
  ColorReference* cr = parent->colorReference();
  switch(parent->currentColor)
  {
    case ColorReference::white:
      setEnabled(true);
      updateWidgetsPrivate(cr->thresholdWhite);
      break;
    default:
      setEnabled(false);
  }
  ignoreUpdateColorReference = false;
}

void ThresholdSelector::setEnabled(bool value)
{
  slider->setEnabled(value);
  lineEdit->setEnabled(value);
}


/* ---------------- Private ---------------- */
void ThresholdSelector::updateColorReference(const int value)
{
  if(ignoreUpdateColorReference)
  {
    return;
  }
  
  ColorReference* cr = parent->colorReference();
  switch(parent->currentColor)
  {
    case ColorReference::white:
      updateColorReference(value, cr->thresholdWhite);
      break;
    default:
      return;
  }
  cr->changed = true;
}


/* ---------------- Slots ---------------- */
void ThresholdSelector::sliderChanged(int value)
{
  lineEdit->setText(QString::number(value));
  updateColorReference(value);
}

void ThresholdSelector::lineEditChanged(QString value)
{
  slider->setValue(value.toInt());
}
