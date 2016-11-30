/**
 * File:   ColorCalibrationView.cpp
 * @author marcel
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 *
 * Created on June 25, 2013, 8:13 PM
 */

#include "ColorCalibrationView.h"
#include "Controller/Views/ImageView.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
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
  currentColor = (FieldColors::Color)settings.value("CurrentColor", FieldColors::green).toInt();
  settings.endGroup();

  // If other views are already open, use their color settings
  for(ImageView* view : colorCalibrationView.console.segmentedImageViews)
    if(view->widget)
      currentColor = view->widget->getDrawnColor();

  y = new YSelector("Y", this, 0, 255);
  h = new HSelector("H", this, 0, 255);
  s = new SSelector("S", this, 0, 255);

  thresholdY = new ThresholdSelector("Threshold", this, 0, 255);

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(y);
  layout->addWidget(h);
  layout->addWidget(s);
  layout->addWidget(thresholdY);
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

  if(currentColor == FieldColors::none || currentColor == FieldColors::white || currentColor == FieldColors::black)
  {
    y->setVisible(false);
    h->setVisible(false);
    s->setVisible(false);
    thresholdY->setVisible(true);
    thresholdY->setEnabled(true);
    thresholdY->updateWidgets();
  }
  else
  {
    thresholdY->setVisible(false);
    thresholdY->setEnabled(false);
    y->setVisible(true);
    h->setVisible(true);
    s->setVisible(true);
    y->setEnabled(true);
    h->setEnabled(true);
    s->setEnabled(true);
    y->updateWidgets();
    h->updateWidgets();
    s->updateWidgets();
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
  QAction* expandColorAct = new QAction(QIcon(":/Icons/rainbowcursor.png"), tr("Toggle &Expand Color Mode"), menu);

  const_cast<ColorCalibrationWidget*>(this)->currentCalibrationChanged();
  expandColorAct->setCheckable(true);
  expandColorAct->setChecked(expandColorMode);

  connect(saveColorCalibration, SIGNAL(triggered()), this, SLOT(saveColorCalibration()));
  connect(undoAction, SIGNAL(triggered()), this, SLOT(undoColorCalibration()));
  connect(redoAction, SIGNAL(triggered()), this, SLOT(redoColorCalibration()));
  connect(expandColorAct, SIGNAL(triggered()), this, SLOT(expandColorAct()));

  menu->addAction(saveColorCalibration);
  menu->addAction(undoAction);
  menu->addAction(redoAction);
  menu->addAction(expandColorAct);
  menu->addSeparator();

  QAction* colorButtons[FieldColors::numOfColors] =
  {
    new QAction(QIcon(":/Icons/orange.png"), tr("Calibrate &Saturation"), menu),
    new QAction(QIcon(":/Icons/white.png"), tr("Calibrate &White"), menu),
    new QAction(QIcon(":/Icons/black.png"), tr("Calibrate &Black"), menu),
    new QAction(QIcon(":/Icons/green.png"), tr("Calibrate &Green"), menu),
    new QAction(QIcon(":/Icons/ownColor.png"), tr("Calibrate &Own Color"), menu),
    new QAction(QIcon(":/Icons/opponentColor.png"), tr("Calibrate &Opponent Color"), menu)
  };

  QActionGroup* colorGroup = new QActionGroup(menu);
  QSignalMapper* signalMapper = new QSignalMapper(const_cast<ColorCalibrationWidget*>(this));
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(colorAct(int)));
  for(int i = 0; i < FieldColors::numOfColors; ++i)
  {
    signalMapper->setMapping(colorButtons[i], i);
    connect(colorButtons[i], SIGNAL(triggered()), signalMapper, SLOT(map()));
    colorGroup->addAction(colorButtons[i]);
    colorButtons[i]->setCheckable(true);
    colorButtons[i]->setChecked(currentColor == i);
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
  updateWidgets((FieldColors::Color) color);
  currentCalibrationChanged();

  for(auto& view : colorCalibrationView.console.actualImageViews)
    if(view.second->widget)
      view.second->widget->setDrawnColor(currentColor);
}

void ColorCalibrationWidget::setUndoRedo()
{
  bool enableUndo;
  bool enableRedo;

  if(currentColor == FieldColors::none || currentColor == FieldColors::white || currentColor == FieldColors::black)
  {
    enableUndo = historyNonColor[currentColor].undoable();
    enableRedo = historyNonColor[currentColor].redoable();
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
  if(currentColor == FieldColors::none)
    historyNonColor[FieldColors::none].add(colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
  else if(currentColor == FieldColors::white)
    historyNonColor[FieldColors::white].add(colorCalibrationView.console.colorCalibration.minYWhite);
  else if(currentColor == FieldColors::black)
    historyNonColor[FieldColors::black].add(colorCalibrationView.console.colorCalibration.maxYBlack);
  else
    historyColors[currentColor - FieldColors::numOfNonColors].add(colorCalibrationView.console.colorCalibration[currentColor]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::undoColorCalibration()
{
  if(currentColor == FieldColors::none)
    historyNonColor[FieldColors::none].undo(colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
  else if(currentColor == FieldColors::white)
    historyNonColor[FieldColors::white].undo(colorCalibrationView.console.colorCalibration.minYWhite);
  else if(currentColor == FieldColors::black)
    historyNonColor[FieldColors::black].undo(colorCalibrationView.console.colorCalibration.maxYBlack);
  else
    historyColors[currentColor - FieldColors::numOfNonColors].undo(colorCalibrationView.console.colorCalibration[currentColor]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::redoColorCalibration()
{
  if(currentColor == FieldColors::none)
    historyNonColor[FieldColors::none].redo(colorCalibrationView.console.colorCalibration.maxNonColorSaturation);
  else if(currentColor == FieldColors::white)
    historyNonColor[FieldColors::white].redo(colorCalibrationView.console.colorCalibration.minYWhite);
  if(currentColor == FieldColors::black)
    historyNonColor[FieldColors::black].redo(colorCalibrationView.console.colorCalibration.maxYBlack);
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
