/**
 * @file Controller/Views/ImageView.cpp
 *
 * Implementation of class ImageView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author Colin Graf
 */

#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QPinchGesture>
#include <QWidget>
#include <QSettings>
#include <QSignalMapper>
#include <QMenu>
#include <sstream>

#include "ImageView.h"
#include "ColorCalibrationView/ColorCalibrationView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"
#include "Tools/ColorModelConversions.h"
#include <QFileDialog>

ImageView::ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, bool upperCam, float gain) :
  upperCam(upperCam), widget(0), fullName(fullName), icon(":/Icons/tag_green.png"), console(console),
  background(background), name(name),
  segmented(segmented),
  gain(gain), isActImage(strcmp(name.c_str(), "act") == 0) {}

SimRobot::Widget* ImageView::createWidget()
{
  widget = new ImageWidget(*this);
  return widget;
}

ImageWidget::ImageWidget(ImageView& imageView) :
  imageView(imageView),
  imageData(0), imageWidth(Image::maxResolutionWidth), imageHeight(Image::maxResolutionHeight),
  lastImageTimeStamp(0), lastColorTableTimeStamp(0), lastDrawingsTimeStamp(0),
  dragStart(-1, -1), zoom(1.f), offset(0, 0), headControlMode(false),
  drawnColor(ColorClasses::none), drawAllColors(true), undoAction(nullptr), redoAction(nullptr)
{
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  grabGesture(Qt::PinchGesture);
  setAttribute(Qt::WA_AcceptTouchEvents);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  zoom = (float)settings.value("Zoom", 1.).toDouble();
  offset = settings.value("Offset", QPoint()).toPoint();
  drawnColor = (ColorClasses::Color) settings.value("DrawnColor", 1).toInt();
  drawAllColors = settings.value("DrawAllColors", true).toBool();
  settings.endGroup();

  // If other views are already open, use their color settings
  for(ImageView* view : imageView.console.segmentedImageViews)
    if(view->widget)
    {
      drawnColor = view->widget->drawnColor;
      drawAllColors = view->widget->drawAllColors;
    }

  ColorCalibrationView* colorView = imageView.console.colorCalibrationView;
  if(colorView && colorView->widget)
    drawnColor = colorView->widget->currentColor;
}

ImageWidget::~ImageWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  settings.setValue("Zoom", (double)zoom);
  settings.setValue("Offset", offset);
  settings.setValue("DrawnColor", drawnColor);
  settings.setValue("DrawAllColors", drawAllColors);
  settings.endGroup();

  imageView.widget = 0;

  if(imageData)
    delete imageData;
}

void ImageWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  paint(painter);
  painter.end();
}

void ImageWidget::paint(QPainter& painter)
{
  SYNC_WITH(imageView.console);

  const Image* image = 0;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);
  if(i != currentImages.end())
  {
    image = i->second.image;
    imageWidth = image->width;
    imageHeight = image->height;
  }
  else if(!currentImages.empty())
  {
    imageWidth = currentImages.begin()->second.image->width;
    imageHeight = currentImages.begin()->second.image->height;
  }

  const QSize& size = painter.window().size();
  float xScale = float(size.width()) / float(imageWidth);
  float yScale = float(size.height()) / float(imageHeight);
  scale = xScale < yScale ? xScale : yScale;
  scale *= zoom;
  float imageXOffset = (float(size.width()) - float(imageWidth) * scale) * 0.5f + offset.x() * scale;
  float imageYOffset = (float(size.height()) - float(imageHeight) * scale) * 0.5f + offset.y() * scale;

  painter.setTransform(QTransform(scale, 0, 0, scale, imageXOffset, imageYOffset));

  if(image)
    paintImage(painter, *image);
  else
    lastImageTimeStamp = 0;

  paintDrawings(painter);
}

void ImageWidget::paintDrawings(QPainter& painter)
{
  const QTransform baseTrans = painter.transform();
  const std::list<std::string>& drawings = imageView.console.imageViews[imageView.name];
  for(const std::string& drawing : drawings)
  {
    auto& camDrawings = imageView.upperCam ? imageView.console.upperCamImageDrawings : imageView.console.lowerCamImageDrawings;
    auto debugDrawing = camDrawings.find(drawing);
    if(debugDrawing != camDrawings.end())
    {
      PaintMethods::paintDebugDrawing(painter, debugDrawing->second, baseTrans);
      if(debugDrawing->second.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawing->second.timeStamp;
    }
    auto& motinoDrawings = imageView.console.motionImageDrawings;
    debugDrawing = motinoDrawings.find(drawing);
    if(debugDrawing != motinoDrawings.end())
    {
      PaintMethods::paintDebugDrawing(painter, debugDrawing->second, baseTrans);
      if(debugDrawing->second.timeStamp > lastDrawingsTimeStamp)
        lastDrawingsTimeStamp = debugDrawing->second.timeStamp;
    }
  }
  painter.setTransform(baseTrans);
}

void ImageWidget::copyImage(const Image& srcImage)
{
  int width = srcImage.width;
  int height = srcImage.height;

  unsigned* p = (unsigned*) imageData->bits();
  int r, g, b;
  int yImage, uImage, vImage;
  for(int y = 0; y < height; ++y)
  {
    const Image::Pixel* cur = &srcImage[y][0];
    const Image::Pixel* end = cur + width;
    for(; cur < end; ++cur)
    {
      yImage = int(cur->y) << 14;
      uImage = int(cur->cr) - 128;
      vImage = int(cur->cb) - 128;

      r = (yImage + 22972 * uImage) >> 14;
      g = (yImage - 5662 * vImage - 11706 * uImage) >> 14;
      b = (yImage + 29016 * vImage) >> 14;

      *p++ = (r < 0 ? 0 : r > 255 ? 255 : r) << 16 |
             (g < 0 ? 0 : g > 255 ? 255 : g) << 8 |
             (b < 0 ? 0 : b > 255 ? 255 : b) |
             0xff000000;
    }
  }
  if(imageView.gain != 1.f)
  {
    unsigned* p = (unsigned*) imageData->bits();
    float gain = imageView.gain;
    for(unsigned* pEnd = p + width * height; p < pEnd; ++p)
    {
      int r = (int)(gain * (float)((*p >> 16) & 0xff));
      int g = (int)(gain * (float)((*p >> 8) & 0xff));
      int b = (int)(gain * (float)((*p) & 0xff));

      *p++ = (r < 0 ? 0 : r > 255 ? 255 : r) << 16 |
             (g < 0 ? 0 : g > 255 ? 255 : g) << 8 |
             (b < 0 ? 0 : b > 255 ? 255 : b) |
             0xff000000;
    }
  }
}

void ImageWidget::copyImageSegmented(const Image& srcImage)
{
  int width = srcImage.width;
  int height = srcImage.height;

  static const unsigned baseColors[] =
  {
    0xffffffff, //white
    0xff00ff00, //green
    0xff0000ff, //blue
    0xffff0000, //red
    0xffff7f00, //orange
    0xff000000  //black
  };

  static unsigned displayColors[1 << (ColorClasses::numOfColors - 1)];
  if(!displayColors[0])
  {
    union
    {
      unsigned color;
      unsigned char channels[4];
    } baseColor;

    displayColors[0] = 0xff7f7f7f; //grey
    for(int colors = 1; colors < 1 << (ColorClasses::numOfColors - 1); ++colors)
    {
      int count = 0;
      for(int i = 0; i < ColorClasses::numOfColors - 1; ++i)
        if(colors & 1 << i)
          ++count;
      unsigned mixed = 0;
      for(int i = 0; i < ColorClasses::numOfColors - 1; ++i)
        if(colors & 1 << i)
        {
          baseColor.color = baseColors[i];
          for(int j = 0; j < 4; ++j)
            baseColor.channels[j] /= count;
          mixed += baseColor.color;
        }
      displayColors[colors] = mixed;
    }
  }

  unsigned* p = (unsigned*) imageData->bits();
  const unsigned char drawnColors = (unsigned char)(drawAllColors ? ~0 : 1 << (drawnColor - 1));
  const ColorTable& ct = imageView.console.colorTable;
  for(int y = 0; y < height; ++y)
    for(const Image::Pixel* cur = &srcImage[y][0], * end = cur + width; cur < end; ++cur)
      *p++ = displayColors[ct[*cur].colors & drawnColors];
}

void ImageWidget::paintImage(QPainter& painter, const Image& srcImage)
{
  // make sure we have a buffer
  if(!imageData || imageWidth != imageData->width() || imageHeight != imageData->height())
  {
    if(imageData)
      delete imageData;
    imageData = new QImage(imageWidth, imageHeight, QImage::Format_RGB32);
  }

  if(srcImage.timeStamp != lastImageTimeStamp || imageView.segmented)
  {
    if(imageView.segmented)
      copyImageSegmented(srcImage);
    else
      copyImage(srcImage);

    lastImageTimeStamp = srcImage.timeStamp;
    if(imageView.segmented)
      lastColorTableTimeStamp = imageView.console.colorTableTimeStamp;
  }

  painter.drawImage(QRectF(0, 0, imageWidth, imageHeight), *imageData);
}

bool ImageWidget::needsRepaint() const
{
  SYNC_WITH(imageView.console);
  Image* image = 0;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator j = currentImages.find(imageView.background);
  if(j != currentImages.end())
    image = j->second.image;

  if(!image)
  {
    const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
    for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
    {
      const DebugDrawing& debugDrawing(imageView.upperCam ? imageView.console.upperCamImageDrawings[*i] : imageView.console.lowerCamImageDrawings[*i]);
      if(debugDrawing.timeStamp > lastDrawingsTimeStamp)
        return true;
    }
    return lastImageTimeStamp != 0;
  }
  else
    return image->timeStamp != lastImageTimeStamp ||
           (imageView.segmented && imageView.console.colorTableTimeStamp != lastColorTableTimeStamp);
}

void ImageWidget::window2viewport(QPoint& point)
{
  const QSize& size(this->size());
  float xScale = float(size.width()) / float(imageWidth);
  float yScale = float(size.height()) / float(imageHeight);
  float scale = xScale < yScale ? xScale : yScale;
  scale *= zoom;
  float xOffset = (float(size.width()) - float(imageWidth) * scale) * 0.5f + offset.x() * scale;
  float yOffset = (float(size.height()) - float(imageHeight) * scale) * 0.5f + offset.y() * scale;
  point = QPoint(static_cast<int>((point.x() - xOffset) / scale),
                 static_cast<int>((point.y() - yOffset) / scale));
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);
  SYNC_WITH(imageView.console);
  QPoint pos(event->pos());

  if(dragStart != QPoint(-1, -1))
  {
    offset = dragStartOffset + (pos - dragStart) / scale;
    QWidget::update();
    return;
  }

  window2viewport(pos);

  const char* text = 0;
  const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
  for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
  {
    if(imageView.upperCam)
      text = imageView.console.upperCamImageDrawings[*i].getTip(pos.rx(), pos.ry());
    else
      text = imageView.console.lowerCamImageDrawings[*i].getTip(pos.rx(), pos.ry());

    if(text)
      break;
  }

  if(text)
    setToolTip(QString(text));
  else
  {
    Image* image = 0;
    RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
    RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);
    if(i != currentImages.end())
      image = i->second.image;
    if(image && pos.rx() >= 0 && pos.ry() >= 0 && pos.rx() < image->width && pos.ry() < image->height)
    {
      Image::Pixel& pixel = (*image)[pos.ry()][pos.rx()];
      char color[128];

      static const int factor1 = 29016;
      static const int factor2 = 5662;
      static const int factor3 = 22972;
      static const int factor4 = 11706;

      int yImage = int(pixel.y) << 14;
      int uImage = int(pixel.cr) - 128;
      int vImage = int(pixel.cb) - 128;

      int r = (yImage + factor3 * uImage) >> 14;
      int g = (yImage - factor2 * vImage - factor4 * uImage) >> 14;
      int b = (yImage + factor1 * vImage) >> 14;

      unsigned char h, s, i;
      ColorModelConversions::fromYCbCrToHSI(pixel.y, pixel.cb, pixel.cr, h, s, i);
      sprintf(color, "x=%d, y=%d\ny=%d, cb=%d, cr=%d\nr=%d, g=%d, b=%d\nh=%d, s=%d, i=%d", pos.rx(), pos.ry(), pixel.y, pixel.cb, pixel.cr, r, g, b, h, s, i);
      setToolTip(QString(color));
    }
    else
      setToolTip(QString());
  }
}

void ImageWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  if(event->button() == Qt::LeftButton || event->button() == Qt::MidButton)
  {
    dragStart = event->pos();
    dragStartOffset = offset;
  }
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);
  QPoint pos = QPoint(event->pos());
  if(dragStart != pos && dragStart != QPoint(-1, -1))
  {
    dragStart = QPoint(-1, -1);
    QWidget::update();
    return;
  }
  dragStart = QPoint(-1, -1);
  window2viewport(pos);
  Vector2i v = Vector2i(pos.x(), pos.y());
  if(event->modifiers() & Qt::ShiftModifier)
  {
    if(!headControlMode)
    {
      imageView.console.handleConsole("mr HeadMotionRequest ManualHeadMotionProvider");
      headControlMode = true;
    }
    std::stringstream command;
    command << "set parameters:ManualHeadMotionProvider xImg = " << v.x()
            << "; yImg = " << v.y()
            << "; camera = " << (imageView.upperCam ? "upper" : "lower") << ";";
    imageView.console.handleConsole(command.str());
  }
  {
    SYNC_WITH(imageView.console);
    if(!(event->modifiers() & Qt::ShiftModifier))
    {
      if((!event->modifiers() || (event->modifiers() & Qt::ControlModifier))
         && imageView.console.colorCalibrationView
         && imageView.console.colorCalibrationView->widget
         && imageView.console.colorCalibrationView->widget->expandColorMode)
      {
        Image* image = 0;
        RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
        RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);
        if(i != currentImages.end())
          image = i->second.image;
        if(image && pos.x() >= 0 && pos.y() >= 0 && pos.x() < image->width && pos.y() < image->height)
          imageView.console.colorCalibrationView->widget->expandCurrentColor((*image)[pos.y()][pos.x()], event->modifiers() & Qt::ControlModifier);
      }
      else if(event->modifiers() & Qt::ControlModifier)
      {
        ImageViewAdapter::fireClick(imageView.name, v, imageView.upperCam, false);
      }
      else
        ImageViewAdapter::fireClick(imageView.name, v, imageView.upperCam, true);
    }
  }
}

void ImageWidget::keyPressEvent(QKeyEvent* event)
{
  switch(event->key())
  {
    case Qt::Key_PageUp:
    case Qt::Key_Plus:
      event->accept();
      if(zoom < 3.f)
        zoom += 0.1f;
      if(zoom > 3.f)
        zoom = 3.f;
      QWidget::update();
      break;
    case Qt::Key_PageDown:
    case Qt::Key_Minus:
      event->accept();
      if(zoom > 0.1f)
        zoom -= 0.1f;
      QWidget::update();
      break;
    case Qt::Key_Up:
      offset += QPoint(0, 20);
      QWidget::update();
      break;
    case Qt::Key_Down:
      offset += QPoint(0, -20);
      QWidget::update();
      break;
    case Qt::Key_Left:
      offset += QPoint(20, 0);
      QWidget::update();
      break;
    case Qt::Key_Right:
      offset += QPoint(-20, 0);
      QWidget::update();
      break;
    default:
      QWidget::keyPressEvent(event);
      break;
  }
}

bool ImageWidget::event(QEvent* event)
{
  if(event->type() == QEvent::Gesture)
  {
    QPinchGesture* pinch = static_cast<QPinchGesture*>(static_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture));
    if(pinch && (pinch->changeFlags() & QPinchGesture::ScaleFactorChanged))
    {
      QPoint before(static_cast<int>(pinch->centerPoint().x()),
                    static_cast<int>(pinch->centerPoint().y()));
      window2viewport(before);
      scale /= zoom;
      zoom *= pinch->scaleFactor() / pinch->lastScaleFactor();
      if(zoom > 3.f)
        zoom = 3.f;
      else if(zoom < 0.1f)
        zoom = 0.1f;
      scale *= zoom;
      QPoint after(static_cast<int>(pinch->centerPoint().x()),
                   static_cast<int>(pinch->centerPoint().y()));
      window2viewport(after);
      offset -= before - after;
      QWidget::update();
      return true;
    }
  }
  return QWidget::event(event);
}

void ImageWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);
#ifndef OSX
  zoom += 0.1 * event->delta() / 120;
  if(zoom > 3.f)
    zoom = 3.f;
  else if(zoom < 0.1f)
    zoom = 0.1f;
  QWidget::update();
#else
  int step = static_cast<int>(event->delta() / (scale * 2.f));
  offset += event->orientation() == Qt::Horizontal ? QPoint(step, 0) : QPoint(0, step);
  if(step)
    QWidget::update();
#endif
}

void ImageWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
  zoom = 1.f;
  offset.setX(0);
  offset.setY(0);
  QWidget::update();
}

QMenu* ImageWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Image"));

  const bool enableColorButtons = imageView.segmented || (imageView.console.colorCalibrationView
                                  && imageView.console.colorCalibrationView->widget
                                  && imageView.console.colorCalibrationView->widget->expandColorMode);

  const_cast<ImageWidget*>(this)->undoAction = new WorkAroundAction(&const_cast<ImageWidget*>(this)->undoAction,
      QIcon(":/Icons/edit_undo.png"), tr("&Undo Color Calibration"), menu);
  const_cast<ImageWidget*>(this)->redoAction = new WorkAroundAction(&const_cast<ImageWidget*>(this)->redoAction,
      QIcon(":/Icons/edit_redo.png"), tr("&Redo Color Calibration"), menu);
  QAction* allColorsAct = new QAction(QIcon(":/Icons/allColors.png"), tr("Show &All Colors"), menu);

  undoAction->setEnabled(false);
  redoAction->setEnabled(false);
  ColorCalibrationView* colorView = imageView.console.colorCalibrationView;
  if(colorView && colorView->widget)
    colorView->widget->setUndoRedo();

  allColorsAct->setCheckable(true);
  allColorsAct->setChecked(drawAllColors);

  allColorsAct->setEnabled(enableColorButtons);

  connect(allColorsAct, SIGNAL(triggered()), this, SLOT(allColorsAct()));

  menu->addAction(undoAction);
  menu->addAction(redoAction);
  menu->addAction(allColorsAct);
  menu->addSeparator();

  QAction* colorButtons[ColorClasses::numOfColors - 1] =
  {
    new QAction(QIcon(":/Icons/white.png"), tr("Show Only &White"), menu),
    new QAction(QIcon(":/Icons/green.png"), tr("Show Only &Green"), menu),
    new QAction(QIcon(":/Icons/blue.png"), tr("Show Only &Blue"), menu),
    new QAction(QIcon(":/Icons/red.png"), tr("Show Only &Red"), menu),
    new QAction(QIcon(":/Icons/orange.png"), tr("Show Only &Orange"), menu),
    new QAction(QIcon(":/Icons/black.png"), tr("Show Only &Black"), menu)
  };

  QActionGroup* colorGroup = new QActionGroup(menu);
  QSignalMapper* signalMapper = new QSignalMapper(const_cast<ImageWidget*>(this));
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(colorAct(int)));
  for(int i = 0; i < ColorClasses::numOfColors - 1; ++i)
  {
    signalMapper->setMapping(colorButtons[i], i + 1);
    connect(colorButtons[i], SIGNAL(triggered()), signalMapper, SLOT(map()));
    colorGroup->addAction(colorButtons[i]);
    colorButtons[i]->setCheckable(true);
    colorButtons[i]->setChecked(drawnColor == i + 1);
    colorButtons[i]->setEnabled(enableColorButtons);
    menu->addAction(colorButtons[i]);
  }

  menu->addSeparator();

  QAction* saveImgAct = new QAction(tr("&Save Image"), menu);
  connect(saveImgAct, SIGNAL(triggered()), this, SLOT(saveImg()));
  menu->addAction(saveImgAct);

  return menu;
}

void ImageWidget::saveImg()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this,
                     tr("Save as PNG"), settings.value("ExportDirectory", "").toString(), tr("(*.png)"));
  if(fileName.isEmpty())
    return;
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());


  SYNC_WITH(imageView.console);

  const Image* image = 0;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);
  if(i != currentImages.end())
  {
    image = i->second.image;
    imageWidth = image->width;
    imageHeight = image->height;
  }
  if(image)
  {
    QPixmap pixmap(image->width, image->height);
    QPainter painter(&pixmap);
    paintImage(painter, *image);
    paintDrawings(painter);
    pixmap.save(fileName, "PNG");
  }
}

void ImageWidget::allColorsAct()
{
  drawAllColors ^= true;
  for(auto& view : imageView.console.actualImageViews)
    if(view.second->widget)
      view.second->widget->setDrawAllColors(drawAllColors);
}

void ImageWidget::colorAct(int color)
{
  for(auto& view : imageView.console.actualImageViews)
    if(view.second->widget)
      view.second->widget->setDrawnColor((ColorClasses::Color) color);

  ColorCalibrationView* colorView = imageView.console.colorCalibrationView;
  if(colorView && colorView->widget)
  {
    colorView->widget->updateWidgets((ColorClasses::Color) color);
    colorView->widget->currentCalibrationChanged();
  }
}

void ImageWidget::setUndoRedo(const bool enableUndo, const bool enableRedo)
{
  if(undoAction && redoAction)
  {
    ColorCalibrationView* colorView = imageView.console.colorCalibrationView;
    disconnect(undoAction, 0, 0, 0);
    disconnect(redoAction, 0, 0, 0);
    connect(undoAction, SIGNAL(triggered()), colorView->widget, SLOT(undoColorCalibration()));
    connect(redoAction, SIGNAL(triggered()), colorView->widget, SLOT(redoColorCalibration()));
    undoAction->setEnabled(enableUndo);
    redoAction->setEnabled(enableRedo);
  }
}
