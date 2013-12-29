/*
 * File:   CameraCalibrationView.cpp
 * Author: marcel
 *
 * Created on June 25, 2013, 8:13 PM
 */

#include "ColorCalibrationView.h"
#include <QVBoxLayout>

/* ------------------- View -------------------  */
ColorCalibrationView::ColorCalibrationView(const QString& fullName, RobotConsole& console)
  : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), widget(nullptr)
{}

SimRobot::Widget* ColorCalibrationView::createWidget()
{
  widget = new ColorCalibrationWidget(*this);
  return widget;
}

const QString& ColorCalibrationView::getFullName() const
{
  return fullName;
}

const QIcon* ColorCalibrationView::getIcon() const
{
  return &icon;
}

/* ------------------- Widget -------------------  */
ColorCalibrationWidget::ColorCalibrationWidget(ColorCalibrationView& colorCalibrationView)
: colorCalibrationView(colorCalibrationView), currentColor(ColorReference::none)
{
  const unsigned int decimals = 2;
  hue = new HueSelector("Hue", this, 0.f, pi2, decimals);
  saturation = new SaturationSelector("Saturation", this, 0, 1.f, decimals);
  value = new ValueSelector("Value", this, 0.f, 1.f, decimals);
  
  minR  = new MinRSelector("Minimum red", this, 0, 254);
  minR->setVisible(false);
  minB  = new MinBSelector("Minimum blue", this, 0, 254);
  minB->setVisible(false);
  minRB = new MinRBSelector("Minimum red + blue", this, 0, 508);
  minRB->setVisible(false);
  
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(hue);
  layout->addWidget(value);
  layout->addWidget(saturation);
  layout->addWidget(minR);
  layout->addWidget(minB);
  layout->addWidget(minRB);
  setLayout(layout);
}

ColorCalibrationWidget::~ColorCalibrationWidget()
{
  colorCalibrationView.widget = nullptr;
}

QWidget* ColorCalibrationWidget::getWidget()
{
  return this;
}

void ColorCalibrationWidget::update()
{
  SYNC_WITH(colorCalibrationView.console);
}

void ColorCalibrationWidget::updateWidgets(unsigned int currentColor)
{
  this->currentColor = currentColor;
  
  if (currentColor == ColorReference::white)
  {
    hue->setVisible(false);
    saturation->setVisible(false);
    value->setVisible(false);
    minR->setVisible(true);
    minB->setVisible(true);
    minRB->setVisible(true);
    minR->updateWidgets();
    minB->updateWidgets();
    minRB->updateWidgets();
  } 
  else
  {
    minR->setVisible(false);
    minB->setVisible(false);
    minRB->setVisible(false);
    hue->setVisible(true);
    saturation->setVisible(true);
    value->setVisible(true);
    hue->updateWidgets();
    saturation->updateWidgets();
    value->updateWidgets();
  }
}

ColorReference* ColorCalibrationWidget::colorReference() const
{
  return &colorCalibrationView.console.colorReference;
}
