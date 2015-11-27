/*
 * File:   ColorCalibrationView.cpp
 * @author marcel
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 *
 * Created on June 25, 2013, 8:13 PM
 */

#include "ColorCalibrationView.h"
#include "Controller/Views/ImageView.h"
#include "Tools/ColorModelConversions.h"
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
  : colorCalibrationView(colorCalibrationView), currentColor(ColorClasses::none), expandColorMode(false), timeStamp(0),
    redoAction(nullptr), undoAction(nullptr)
{
  setFocusPolicy(Qt::StrongFocus);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(colorCalibrationView.getFullName());
  currentColor = (ColorClasses::Color) settings.value("CurrentColor", ColorClasses::green).toInt();
  settings.endGroup();

  // If other views are already open, use their color settings
  for(ImageView* view : colorCalibrationView.console.segmentedImageViews)
    if(view->widget)
      currentColor = view->widget->getDrawnColor();

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
  {
    updateWidgets(currentColor);
    timeStamp = colorCalibrationView.console.colorTableTimeStamp;
  }
}

void ColorCalibrationWidget::updateWidgets(ColorClasses::Color currentColor)
{
  this->currentColor = currentColor;

  if(currentColor == ColorClasses::none)
  {
    if(hue->isVisible())
    {
      minR->setVisible(false);
      minB->setVisible(false);
      minRB->setVisible(false);
    }
    else
    {
      hue->setVisible(false);
      saturation->setVisible(false);
      intensity->setVisible(false);
    }

    hue->setEnabled(false);
    saturation->setEnabled(false);
    intensity->setEnabled(false);
    minR->setEnabled(false);
    minB->setEnabled(false);
    minRB->setEnabled(false);
  }
  else if(currentColor == ColorClasses::white)
  {
    hue->setVisible(false);
    saturation->setVisible(false);
    intensity->setVisible(false);
    minR->setVisible(true);
    minB->setVisible(true);
    minRB->setVisible(true);
    minR->setEnabled(true);
    minB->setEnabled(true);
    minRB->setEnabled(true);
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
    hue->setEnabled(true);
    saturation->setEnabled(true);
    intensity->setEnabled(true);
    hue->updateWidgets();
    saturation->updateWidgets();
    intensity->updateWidgets();
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

  QAction* colorButtons[ColorClasses::numOfColors - 1] =
  {
    new QAction(QIcon(":/Icons/white.png"), tr("Calibrate &White"), menu),
    new QAction(QIcon(":/Icons/green.png"), tr("Calibrate &Green"), menu),
    new QAction(QIcon(":/Icons/blue.png"), tr("Calibrate &Blue"), menu),
    new QAction(QIcon(":/Icons/red.png"), tr("Calibrate &Red"), menu),
    new QAction(QIcon(":/Icons/orange.png"), tr("Calibrate &Orange"), menu),
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

void ColorCalibrationWidget::colorAct(int color)
{
  updateWidgets((ColorClasses::Color) color);
  currentCalibrationChanged();

  for(auto& view : colorCalibrationView.console.actualImageViews)
    if(view.second->widget)
      view.second->widget->setDrawnColor(currentColor);
}

void ColorCalibrationWidget::setUndoRedo()
{
  bool enableUndo;
  bool enableRedo;

  if(currentColor == ColorClasses::white)
  {
    enableUndo = historyWhite.undoable();
    enableRedo = historyWhite.redoable();
  }
  else
  {
    enableUndo = historyColors[currentColor].undoable();
    enableRedo = historyColors[currentColor].redoable();
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
  if(currentColor == ColorClasses::white)
    historyWhite.add(colorCalibrationView.console.colorCalibration.white);
  else
    historyColors[currentColor].add(colorCalibrationView.console.colorCalibration.ranges[currentColor - 2]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::undoColorCalibration()
{
  if(currentColor == ColorClasses::white)
    historyWhite.undo(colorCalibrationView.console.colorCalibration.white);
  else
    historyColors[currentColor].undo(colorCalibrationView.console.colorCalibration.ranges[currentColor - 2]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::redoColorCalibration()
{
  if(currentColor == ColorClasses::white)
    historyWhite.redo(colorCalibrationView.console.colorCalibration.white);
  else
    historyColors[currentColor].redo(colorCalibrationView.console.colorCalibration.ranges[currentColor - 2]);

  updateWidgets(currentColor);
  setUndoRedo();
}

void ColorCalibrationWidget::expandColorAct()
{
  expandColorMode ^= true;
}

void ColorCalibrationWidget::expandCurrentColor(const Image::Pixel& pixel, const bool reduce)
{
  bool changed = false;

  if(currentColor == ColorClasses::white)
  {
    unsigned char r, g, b;
    ColorModelConversions::fromYCbCrToRGB(pixel.y, pixel.cb, pixel.cr, r, g, b);

    int& minR = colorCalibrationView.console.colorCalibration.white.minR;
    int& minB = colorCalibrationView.console.colorCalibration.white.minB;
    int& minRB = colorCalibrationView.console.colorCalibration.white.minRB;

    if(reduce)
    {
      int rDist = 1000, bDist = 1000, rbDist = 1000;
      if(r >= minR)
        rDist = r - minR;
      if(b >= minB)
        bDist = b - minB;
      if(r + b >= minRB)
        rbDist = (r + b) - minRB;

      changed = rDist != 1000 || bDist != 1000 || rbDist != 1000;

      if(changed)
      {
        if(rDist < bDist && rDist < rbDist)
          minR = std::min(255, r + 1);
        else if(bDist < rDist && bDist < rbDist)
          minB = std::min(255, b + 1);
        else if(rbDist < rDist && rbDist < bDist)
          minRB = std::min(255, (r + b) + 1);
      }
    }
    else
    {
      if(r < minR)
      {
        minR = r;
        changed = true;
      }
      if(b < minB)
      {
        minB = b;
        changed = true;
      }
      if(r + b < minRB)
      {
        minRB = r + b;
        changed = true;
      }
    }
  }
  else
  {
    unsigned char h, s, i;
    ColorModelConversions::fromYCbCrToHSI(pixel.y, pixel.cb, pixel.cr, h, s, i);
    ColorCalibration::HSIRanges& ranges = colorCalibrationView.console.colorCalibration.ranges[currentColor - 2];

    if(reduce)
    {
      int hMin = ranges.hue.min;
      int hMax = ranges.hue.max;
      int sMin = ranges.saturation.min;
      int sMax = ranges.saturation.max;
      int iMin = ranges.intensity.min;
      int iMax = ranges.intensity.max;

      const bool hChanged = reduceColor(h, hMin, hMax, false);
      const bool sChanged = reduceColor(s, sMin, sMax, true);
      const bool iChanged = reduceColor(i, iMin, iMax, true);

      changed = hChanged || sChanged || iChanged;

      if(changed)
      {
        const int hMinChange = hMin != ranges.hue.min ? calcColorValueDistance(hMin, ranges.hue.min) : 1000;
        const int hMaxChange = hMax != ranges.hue.max ? calcColorValueDistance(hMax, ranges.hue.max) : 1000;
        const int sMinChange = sChanged ? calcColorValueDistance(sMin, ranges.saturation.min) : 1000;
        const int iMinChange = iChanged ? calcColorValueDistance(iMin, ranges.intensity.min) : 1000;

        if(hMinChange <= hMaxChange || hMinChange <= sMinChange || hMinChange <= iMinChange)
          ranges.hue.min = hMin;
        else if(hMaxChange <= hMinChange || hMaxChange <= sMinChange || hMaxChange <= iMinChange)
          ranges.hue.max = hMax;
        else if(sMinChange <= hMinChange || sMinChange <= hMaxChange || sMinChange <= iMinChange)
          ranges.saturation.min = sMin;
        else if(iMinChange <= hMinChange || iMinChange <= hMaxChange || iMinChange <= sMinChange)
          ranges.intensity.min = iMin;
      }
    }
    else
    {
      changed |= expandColor(h, ranges.hue.min, ranges.hue.max, false);
      changed |= expandColor(s, ranges.saturation.min, ranges.saturation.max, true);
      changed |= expandColor(i, ranges.intensity.min, ranges.intensity.max, true);
    }
  }

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
