/**
 * File:   ColorCalibrationView.cpp
 * @author marcel
 * @author Andreas Stolpmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "ColorCalibrationView.h"
#include "Controller/Views/ImageView.h"
#include <QVBoxLayout>
#include <QSettings>
#include <QSignalMapper>

/* ------------------- View -------------------  */
ColorCalibrationView::ColorCalibrationView(const QString& fullName, RobotConsole& console) :
  console(console), fullName(fullName), icon(":/Icons/tag_green.png")
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
ColorCalibrationWidget::ColorCalibrationWidget(ColorCalibrationView& colorCalibrationView) :
  colorCalibrationView(colorCalibrationView)
{
  setFocusPolicy(Qt::StrongFocus);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(colorCalibrationView.getFullName());
  settings.endGroup();

  hueField = new HueFieldSelector("Field", this, 0, 255);

  thresholdColor = new ColorSelector("ThresholdColor", this, 0, 255);
  thresholdBlackWhite = new BlackWhiteSelector("ThresholdBlackWhite", this, 0, 255);

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(thresholdColor);
  layout->addWidget(hueField);
  layout->addWidget(thresholdBlackWhite);
  updateWidgets();
  setLayout(layout);
}

ColorCalibrationWidget::~ColorCalibrationWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(colorCalibrationView.getFullName());
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
  if(colorCalibrationView.console.colorCalibrationTimestamp != timestamp)
  {
    updateWidgets();
    timestamp = colorCalibrationView.console.colorCalibrationTimestamp;
  }
}

void ColorCalibrationWidget::updateWidgets()
{
  hueField->setVisible(true);
  thresholdColor->setVisible(true);
  thresholdBlackWhite->setVisible(true);

  hueField->setEnabled(true);
  thresholdColor->setEnabled(true);
  thresholdBlackWhite->setEnabled(true);

  thresholdColor->updateWidgets();
  thresholdBlackWhite->updateWidgets();
  hueField->updateWidgets();
}

QMenu* ColorCalibrationWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Color Calibration"));

  QAction* saveColorCalibration = new QAction(QIcon(":/Icons/kick_save.png"), tr("&Save Color Calibration"), menu);
  const_cast<ColorCalibrationWidget*>(this)->undoAction = new WorkAroundAction(&const_cast<ColorCalibrationWidget*>(this)->undoAction,
      QIcon(":/Icons/edit_undo.png"), tr("&Undo Color Calibration"), menu);
  const_cast<ColorCalibrationWidget*>(this)->redoAction = new WorkAroundAction(&const_cast<ColorCalibrationWidget*>(this)->redoAction,
      QIcon(":/Icons/edit_redo.png"), tr("&Redo Color Calibration"), menu);

  const_cast<ColorCalibrationWidget*>(this)->currentCalibrationChanged();

  connect(saveColorCalibration, SIGNAL(triggered()), this, SLOT(saveColorCalibration()));
  connect(undoAction, SIGNAL(triggered()), this, SLOT(undoColorCalibration()));
  connect(redoAction, SIGNAL(triggered()), this, SLOT(redoColorCalibration()));

  menu->addAction(saveColorCalibration);
  menu->addAction(undoAction);
  menu->addAction(redoAction);
  menu->addSeparator();

  return menu;
}

void ColorCalibrationWidget::saveColorCalibration()
{
  colorCalibrationView.console.saveColorCalibration();
  if(colorCalibrationView.console.mode == SystemCall::Mode::remoteRobot)
  {
    colorCalibrationView.console.handleConsole("save representation:CameraSettings");
  }
}

void ColorCalibrationWidget::setUndoRedo()
{
  if(undoAction != nullptr)
    undoAction->setEnabled(history.undoable());
  if(redoAction != nullptr)
    redoAction->setEnabled(history.redoable());
}

void ColorCalibrationWidget::currentCalibrationChanged()
{
  history.add(colorCalibrationView.console.colorCalibration);

  updateWidgets();
  setUndoRedo();
}

void ColorCalibrationWidget::undoColorCalibration()
{
  history.undo(colorCalibrationView.console.colorCalibration);
  updateWidgets();
  setUndoRedo();
}

void ColorCalibrationWidget::redoColorCalibration()
{
  history.redo(colorCalibrationView.console.colorCalibration);
  updateWidgets();
  setUndoRedo();
}
