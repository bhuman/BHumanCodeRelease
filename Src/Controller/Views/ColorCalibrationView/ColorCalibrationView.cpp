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
  currentColor = (FieldColors::Color)settings.value("CurrentColor", FieldColors::none).toInt();
  settings.endGroup();

  hue = new HueSelector("Hue", this, 0, 255);
  hueField = new HueFieldSelector("Field", this, 0, 255);

  thresholdColor = new ColorSelector("ThresholdColor", this, 0, 255);
  thresholdBlackWhite = new BlackWhiteSelector("ThresholdBlackWhite", this, 0, 255);

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(thresholdColor);
  layout->addWidget(hueField);
  layout->addWidget(hue);
  layout->addWidget(thresholdBlackWhite);
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
  if(colorCalibrationView.console.colorCalibrationTimeStamp != timeStamp)
  {
    updateWidgets(currentColor);
    timeStamp = colorCalibrationView.console.colorCalibrationTimeStamp;
  }
}

void ColorCalibrationWidget::updateWidgets(FieldColors::Color currentColor)
{
  this->currentColor = currentColor;

  if(currentColor < FieldColors::numOfNonColors)
  {
    hue->setVisible(false);

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
  else
  {
    thresholdColor->setVisible(false);
    thresholdBlackWhite->setVisible(false);
    hueField->setVisible(false);

    hue->setVisible(true);
    hue->setEnabled(true);
    hue->updateWidgets();
  }
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

  QAction* colorButtons[FieldColors::numOfColors - FieldColors::numOfNonColors + 1] =
  {
    new QAction(QIcon(":/Icons/orange.png"), tr("Calibrate &Saturation"), menu),
    new QAction(QIcon(":/Icons/ownColor.png"), tr("Calibrate &Own Color"), menu),
    new QAction(QIcon(":/Icons/opponentColor.png"), tr("Calibrate &Opponent Color"), menu)
  };

  QActionGroup* colorGroup = new QActionGroup(menu);
  QSignalMapper* signalMapper = new QSignalMapper(const_cast<ColorCalibrationWidget*>(this));
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(colorAct(int)));
  for(int i = 0; i < FieldColors::numOfColors - FieldColors::numOfNonColors + 1; ++i)
  {
    signalMapper->setMapping(colorButtons[i], i);
    connect(colorButtons[i], SIGNAL(triggered()), signalMapper, SLOT(map()));
    colorGroup->addAction(colorButtons[i]);
    colorButtons[i]->setCheckable(true);
    colorButtons[i]->setChecked(currentColor == i + FieldColors::numOfNonColors - 1);
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
    colorCalibrationView.console.handleConsole("save representation:CameraSettings");
  }
}

void ColorCalibrationWidget::colorAct(int color)
{
  updateWidgets(static_cast<FieldColors::Color>(color + FieldColors::numOfNonColors - 1));
  currentCalibrationChanged();
}

void ColorCalibrationWidget::setUndoRedo()
{
  bool enableUndo;
  bool enableRedo;

  if(currentColor < FieldColors::numOfNonColors)
  {
    enableUndo = historyBasic.undoable();
    enableRedo = historyBasic.redoable();
  }
  else
  {
    enableUndo = historyColors[currentColor - FieldColors::numOfNonColors].undoable();
    enableRedo = historyColors[currentColor - FieldColors::numOfNonColors].redoable();
  }

  if(undoAction && redoAction)
  {
    undoAction->setEnabled(enableUndo);
    redoAction->setEnabled(enableRedo);
  }

  for(auto& view : colorCalibrationView.console.actualImageViews)
    if(view.second->widget)
      view.second->widget->setUndoRedo(enableUndo, enableRedo);
}

void ColorCalibrationWidget::currentCalibrationChanged()
{
  if(currentColor < FieldColors::numOfNonColors)
  {
    historyBasic.add(colorCalibrationView.console.colorCalibration.copyBasicParameters());
  }
  else
    historyColors[currentColor - FieldColors::numOfNonColors].add(colorCalibrationView.console.colorCalibration[currentColor]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::undoColorCalibration()
{
  if(currentColor < FieldColors::numOfNonColors)
  {
    FieldColors::BasicParameters p(colorCalibrationView.console.colorCalibration.copyBasicParameters());
    historyBasic.undo(p);
    colorCalibrationView.console.colorCalibration.setBasicParameters(p);
  }
  else
    historyColors[currentColor - FieldColors::numOfNonColors].undo(colorCalibrationView.console.colorCalibration[currentColor]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::redoColorCalibration()
{
  if(currentColor < FieldColors::numOfNonColors)
  {
    FieldColors::BasicParameters p(colorCalibrationView.console.colorCalibration.copyBasicParameters());
    historyBasic.redo(p);
    colorCalibrationView.console.colorCalibration.setBasicParameters(p);
  }
  else
    historyColors[currentColor - FieldColors::numOfNonColors].redo(colorCalibrationView.console.colorCalibration[currentColor]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::expandColorAct()
{
  expandColorMode ^= true;
}

void ColorCalibrationWidget::expandCurrentColor(const PixelTypes::YUYVPixel& pixel, const bool reduce)
{
  bool changed = false;

  if(changed)
    currentCalibrationChanged();
}

bool ColorCalibrationWidget::reduceColor(const unsigned char value, int& min, int& max, const bool boolChangeOnlyMin)
{
  if((min < max && (value > max || value < min))
     || (min > max && (value > max && value < min)))
    return false;

  int valueMin = value + (min < max ? 1 : -1);
  int valueMax = value + (min < max ? -1 : 1);

  valueMin = std::min(std::max(0, valueMin), 255);
  valueMax = std::min(std::max(0, valueMax), 255);

  if(boolChangeOnlyMin || calcColorValueDistance(valueMin, min) <= calcColorValueDistance(valueMax, max))
    min = valueMin;
  else
    max = valueMax;

  return true;
}

bool ColorCalibrationWidget::expandColor(const unsigned char value, int& min, int& max, const bool noWrapAround)
{
  if((min < max && (value >= min && value <= max))
     || (min > max && (value >= min || value <= max)))
    return false;

  if(noWrapAround && min <= max)
  {
    if(value < min)
      min = value;
    else if(value > max)
      max = value;
  }
  else
  {
    if(calcColorValueDistance(value, min) <= calcColorValueDistance(value, max))
      min = value;
    else
      max = value;
  }

  return true;
}

int ColorCalibrationWidget::calcColorValueDistance(const int a, const int b) const
{
  int dist = std::abs(a - b);
  if(dist >= 256 / 2)
    dist = (dist - 256) * -1;
  return dist;
}
