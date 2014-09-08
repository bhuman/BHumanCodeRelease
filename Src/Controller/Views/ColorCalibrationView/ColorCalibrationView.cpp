/*
 * File:   ColorCalibrationView.cpp
 * Author: marcel
 *
 * Created on June 25, 2013, 8:13 PM
 */

#include "ColorCalibrationView.h"
#include <QVBoxLayout>
#include <QSettings>
#include <QSignalMapper>

/* ------------------- View -------------------  */
ColorCalibrationView::ColorCalibrationView(const QString& fullName, RobotConsole& console)
  : console(console), widget(nullptr), fullName(fullName), icon(":/Icons/tag_green.png")
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
: colorCalibrationView(colorCalibrationView), timeStamp(0)
{
  setFocusPolicy(Qt::StrongFocus);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(colorCalibrationView.getFullName());
  currentColor = (ColorClasses::Color) settings.value("CurrentColor", ColorClasses::green).toInt();
  settings.endGroup();

  hue = new HueSelector("Hue", this, 0, 255);
  saturation = new SaturationSelector("Saturation", this, 0, 255);
  intensity = new IntensitySelector("Intensity", this, 0, 255);

  minR  = new MinRSelector("Minimum Red", this, 0, 254);
  minB  = new MinBSelector("Minimum Blue", this, 0, 254);
  minRB = new MinRBSelector("Minimum Red + Blue", this, 0, 508);

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(hue);
  layout->addWidget(saturation);
  layout->addWidget(intensity);
  layout->addWidget(minR);
  layout->addWidget(minB);
  layout->addWidget(minRB);
  updateWidgets(currentColor);
  setLayout(layout);
}

ColorCalibrationWidget::~ColorCalibrationWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(colorCalibrationView.getFullName());
  settings.setValue("CurrentColor", currentColor);
  settings.endGroup();
  colorCalibrationView.widget = nullptr;
}

QWidget* ColorCalibrationWidget::getWidget()
{
  return this;
}

void ColorCalibrationWidget::update()
{
  SYNC_WITH(colorCalibrationView.console);
  if(colorCalibrationView.console.colorTableTimeStamp != timeStamp)
    updateWidgets(currentColor);
}

void ColorCalibrationWidget::updateWidgets(ColorClasses::Color currentColor)
{
  this->currentColor = currentColor;

  if(currentColor == ColorClasses::white)
  {
    hue->setVisible(false);
    saturation->setVisible(false);
    intensity->setVisible(false);
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
    intensity->setVisible(true);
    hue->updateWidgets();
    saturation->updateWidgets();
    intensity->updateWidgets();
  }
}

QMenu* ColorCalibrationWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Color Calibration"));

  QAction* saveColorCalibration = new QAction(QIcon(":/Icons/kick_save.png"), tr("&Save Color Calibration"), menu);
  menu->addAction(saveColorCalibration);
  connect(saveColorCalibration, SIGNAL(triggered()), this, SLOT(saveColorCalibration()));

  menu->addSeparator();

  QAction* colorButtons[] =
  {
    new QAction(QIcon(":/Icons/white.png"), tr("Calibrate &White"), menu),
    new QAction(QIcon(":/Icons/green.png"), tr("Calibrate &Green"), menu),
    new QAction(QIcon(":/Icons/blue.png"), tr("Calibrate &Blue"), menu),
    new QAction(QIcon(":/Icons/red.png"), tr("Calibrate &Red"), menu),
    new QAction(QIcon(":/Icons/orange.png"), tr("Calibrate &Orange"), menu),
    new QAction(QIcon(":/Icons/yellow.png"), tr("Calibrate &Yellow"), menu),
    new QAction(QIcon(":/Icons/black.png"), tr("Calibrate &Black"), menu)
  };

  QActionGroup* colorGroup = new QActionGroup(menu);
  QSignalMapper* signalMapper = new QSignalMapper(const_cast<ColorCalibrationWidget*>(this));
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(colorAct(int)));
  for(int i = 0; i < ColorClasses::numOfColors - 1; ++i)
  {
    signalMapper->setMapping(colorButtons[i], i + 1);
    connect(colorButtons[i], SIGNAL(triggered()), signalMapper, SLOT(map()));
    colorGroup->addAction(colorButtons[i]);
    colorButtons[i]->setCheckable(true);
    colorButtons[i]->setChecked(currentColor == i + 1);
    colorButtons[i]->setEnabled(true);
    menu->addAction(colorButtons[i]);
  }

  return menu;
}

void ColorCalibrationWidget::saveColorCalibration()
{
  colorCalibrationView.console.saveColorCalibration();
  if(colorCalibrationView.console.mode == SystemCall::Mode::remoteRobot)
  {
    colorCalibrationView.console.handleConsole("save representation:UpperCameraSettings");
    colorCalibrationView.console.handleConsole("save representation:LowerCameraSettings");
  }
}
