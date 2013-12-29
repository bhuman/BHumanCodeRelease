/**
 * @file Controller/Views/ImageView.cpp
 *
 * Implementation of class ImageView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author Colin Graf
 */

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QSettings>
#include <QMenu>
#include <sstream>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "ImageView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "ColorCalibrationView/ColorCalibrationView.h"


ImageWidget::ImageWidget(ImageView& imageView) :
  imageView(imageView),
  imageData(0), imageWidth(maxResolutionWidth), imageHeight(maxResolutionHeight),
  lastImageTimeStamp(0), lastColorReferenceTimeStamp(0), lastDrawingsTimeStamp(0),
  dragStart(-1, -1), zoom(1.f), offset(0, 0), headControlMode(false)
{
  drawnColors = allColors;
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  zoom = (float)settings.value("Zoom", 1.).toDouble();
  offset = settings.value("Offset", QPoint()).toPoint();
  settings.endGroup();
}

ImageWidget::~ImageWidget()
{
  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  settings.setValue("Zoom", (double)zoom);
  settings.setValue("Offset", offset);
  settings.endGroup();

  if(imageData)
    delete imageData;
}


void ImageWidget::paintEvent(QPaintEvent* event)
{
  painter.begin(this);
  paint(painter);

 /* if(dragStart.x() >= 0)
  {
    QPen pen(QColor(128, 128, 128, 128));
    pen.setWidth(1);
    painter.setPen(pen);
    painter.setBrush(QBrush(Qt::NoBrush));
    painter.drawRect(QRect(dragStart, dragPos));
  }*/

  painter.end();
}

void ImageWidget::paint(QPainter& painter)
{
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

    float scale;
    {
      const QSize& size = painter.window().size();
      float xScale = float(size.width()) / float(imageWidth);
      float yScale = float(size.height()) / float(imageHeight);
      scale = xScale < yScale ? xScale : yScale;
      scale *= zoom;
      imageXOffset = (float(size.width()) - float(imageWidth) * scale) * 0.5f + offset.x() * scale;
      imageYOffset = (float(size.height()) - float(imageHeight) * scale) * 0.5f + offset.y() * scale;

    }

    painter.setTransform(QTransform(scale, 0, 0, scale, imageXOffset, imageYOffset));

    if(image)
      paintImage(painter, *image);
    else
      lastImageTimeStamp = 0;

    paintDrawings(painter);
  }
}

void ImageWidget::paintDrawings(QPainter& painter)
{
  const QTransform baseTrans(painter.transform());
  const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
  for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
  {
    const DebugDrawing& debugDrawing(imageView.upperCam ? imageView.console.upperCamImageDrawings[*i] : imageView.console.lowerCamImageDrawings[*i]);

    PaintMethods::paintDebugDrawing(painter, debugDrawing, baseTrans);
    if(debugDrawing.timeStamp > lastDrawingsTimeStamp)
      lastDrawingsTimeStamp = debugDrawing.timeStamp;

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
      int r = (int)(gain * (float) ((*p >> 16) & 0xff));
      int g = (int)(gain * (float) ((*p >> 8) & 0xff));
      int b = (int)(gain * (float) ((*p) & 0xff));

      *p++ = (r < 0 ? 0 : r > 255 ? 255 : r) << 16 |
              (g < 0 ? 0 : g > 255 ? 255 : g) << 8 |
              (b < 0 ? 0 : b > 255 ? 255 : b) |
              0xff000000;
    }
  }
}

void ImageWidget::copyImageSegmented(const Image& srcImage)
{
  // update the ColorReference if necessary
  unsigned timeStamp = imageView.console.colorReferenceChangedTimeStamp;
  unsigned currentTime = SystemCall::getCurrentSystemTime();
  ColorReference* cr = &imageView.console.colorReference;
  if(cr->changed && timeStamp + 100 < currentTime) {
    cr->update();
    cr->changed = false;
    imageView.console.colorReferenceChangedTimeStamp = currentTime;
  }
  
  int width = srcImage.width;
  int height = srcImage.height;

  static const unsigned baseColors[] =
  {
    0xffff7f00,
    0xffffff00,
    0xff0000ff,
    0xffffffff,
    0xff00ff00,
    0xff000000,
    0xffff0000
  };

  static unsigned displayColors[1 << (ColorClasses::numOfColors - 1)];
  if(!displayColors[0])
  {
    displayColors[0] = 0xff7f7f7f;
    for(int colors = 1; colors < 1 << (ColorClasses::numOfColors - 1); ++colors)
    {
      int count = 0;
      for(int i = 0; i < ColorClasses::numOfColors - 1; ++i)
        if(colors & 1 << i)
          ++count;
      unsigned mixed = 0;
      for(int i = 0; i < ColorClasses::numOfColors - 1; ++i)
        if(colors & 1 << i)
          mixed += baseColors[i] / count;
      displayColors[colors] = mixed;
    }
  }

  unsigned* p = (unsigned*) imageData->bits();
  for(int y = 0; y < height; ++y)
    for(const Image::Pixel* cur = &srcImage[y][0], * end = cur + width; cur < end; ++cur)
      *p++ = displayColors[imageView.console.colorReference.getColorClasses(cur).colors & drawnColors];
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
      lastColorReferenceTimeStamp = imageView.console.colorReferenceTimeStamp;
  }

  painter.drawImage(QRectF(0, 0, imageWidth, imageHeight), *imageData);
}

bool ImageWidget::needsRepaint() const
{
  SYNC_WITH(imageView.console);
  const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
  for(std::list<std::string>::const_iterator i = drawings.begin(), end = drawings.end(); i != end; ++i)
  {
    const DebugDrawing& debugDrawing(imageView.upperCam ? imageView.console.upperCamImageDrawings[*i] : imageView.console.lowerCamImageDrawings[*i]);
    if(debugDrawing.timeStamp > lastDrawingsTimeStamp)
      return true;
  }
  Image* image = 0;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator j = currentImages.find(imageView.background);
  if(j != currentImages.end())
    image = j->second.image;
  return (image &&
         (image->timeStamp != lastImageTimeStamp ||
         (imageView.segmented && imageView.console.colorReferenceTimeStamp != lastColorReferenceTimeStamp))) ||
         (imageView.segmented && imageView.console.colorReference.changed) ||
         (!image && lastImageTimeStamp);
}

void ImageWidget::window2viewport(QPoint& point)
{
  float scale, xOffset, yOffset;
  {
    const QSize& size(this->size());
    float xScale = float(size.width()) / float(imageWidth);
    float yScale = float(size.height()) / float(imageHeight);
    scale = xScale < yScale ? xScale : yScale;
    scale *= zoom;
    xOffset = (float(size.width()) - float(imageWidth) * scale) * 0.5f + offset.x() * scale;
    yOffset = (float(size.height()) - float(imageHeight) * scale) * 0.5f + offset.y() * scale;
  }
  point = QPoint(static_cast<int>((point.x() - xOffset) / scale),
                 static_cast<int>((point.y() - yOffset) / scale));
}

void ImageWidget::mousePressEvent(QMouseEvent* event)
{
  QWidget::mousePressEvent(event);

  handleHeadControl(*event);
}

void ImageWidget::updateColorCalibrator()
{
  ColorCalibrationView* view = imageView.console.colorCalibrator;
  if (view->widget != nullptr)
  {
    view->widget->updateWidgets(drawnColors);
  }
}

void ImageWidget::handleHeadControl(QMouseEvent& event)
{
  if(headControlMode)
  {
    const int x = event.x() - int(imageXOffset);
    const int y = event.y() - int(imageYOffset);

    if(x < 0 || y < 0) return;


    std::stringstream command;
    command << "set parameters:ManualHeadMotionProvider xImg = " << x
            << "; yImg = " << y
            << "; camera = " << (imageView.upperCam ? "upper" : "lower") << ";";

    imageView.console.handleConsole(command.str());

//
//    MessageQueue tempQ; //temp queue used to build the message
//    std::string debugRequest = theConsole.getDebugRequest(theName);
//    tempQ.out.bin << debugRequest << char(0); //0 means unchanged
//    tempQ.out.finishMessage(idDebugDataChangeRequest);
//
//  theConsole.sendDebugMessage(tempQ.in);
//  setIgnoreUpdates(false);


//    imageView.console.sendDebugMessage();

  }
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
  QWidget::mouseMoveEvent(event);
  {
    SYNC_WITH(imageView.console);
    QPoint pos(event->pos());
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
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QWidget::mouseReleaseEvent(event);

  QPoint posInViewPort = QPoint(event->pos());
  window2viewport(posInViewPort);
  Vector2<int> v = Vector2<int>(posInViewPort.x(), posInViewPort.y());
  ImageViewAdapter::fireClick(imageView.name, v);

  if(dragStart.x() < 0)
    return;

  dragStart = QPoint(-1, -1);
  QWidget::update();
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

void ImageWidget::wheelEvent(QWheelEvent* event)
{
  QWidget::wheelEvent(event);

  zoom += 0.1 * event->delta() / 120;
  if(zoom > 3.f)
    zoom = 3.f;
  else if(zoom < 0.1f)
    zoom = 0.1f;
  QWidget::update();
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

  QAction* headAngleAct  = new QAction(QIcon(":/Icons/headAngle.png"), tr("Move Head &Freely"), menu);
  QAction* camUpAct      = new QAction(QIcon(":/Icons/camUp.png"), tr("&Upper Camera View"), menu);
  QAction* camDownAct    = new QAction(QIcon(":/Icons/camDown.png"), tr("&Low Camera View"), menu);

  QAction* headControlAct = new QAction(QIcon(":/Icons/move.png"), tr("&Head Control Mode"), menu);
  headControlAct->setCheckable(true);
  headControlAct->setChecked(false);
  headControlAct->setToolTip("Control the robot's head by clicking and dragging the image");

  QAction* greenAct      = new QAction(QIcon(":/Icons/green.png"), tr("Show Only &Green"), menu);
  QAction* yellowAct     = new QAction(QIcon(":/Icons/yellow.png"), tr("Show Only &Yellow"), menu);
  QAction* orangeAct     = new QAction(QIcon(":/Icons/orange.png"), tr("Show Only &Orange"), menu);
  QAction* redAct        = new QAction(QIcon(":/Icons/red.png"), tr("Show Only &Red"), menu);
  QAction* blueAct       = new QAction(QIcon(":/Icons/blue.png"), tr("Show Only &Blue"), menu);
  QAction* whiteAct      = new QAction(QIcon(":/Icons/white.png"), tr("Show Only &White"), menu);
  QAction* blackAct      = new QAction(QIcon(":/Icons/black.png"), tr("Show Only Blac&k"), menu);
  QAction* allColorsAct  = new QAction(QIcon(":/Icons/allColors.png"), tr("Show &All Colors"), menu);
  QAction* saveColorCalibration  = new QAction(QIcon(":/Icons/bike_save.png"), tr("&Save Current Color Calibration"), menu);

  connect(headAngleAct  , SIGNAL(triggered()), this, SLOT(headAngle()));
  connect(camUpAct      , SIGNAL(triggered()), this, SLOT(camUpOn()));
  connect(camDownAct    , SIGNAL(triggered()), this, SLOT(camDownOn()));

  connect(headControlAct, SIGNAL(toggled(bool)), this, SLOT(headControlToggled(bool)));

  connect(greenAct, SIGNAL(triggered()), this, SLOT(greenAct()));
  connect(yellowAct, SIGNAL(triggered()), this, SLOT(yellowAct()));
  connect(orangeAct, SIGNAL(triggered()), this, SLOT(orangeAct()));
  connect(redAct, SIGNAL(triggered()), this, SLOT(redAct()));
  connect(blueAct, SIGNAL(triggered()), this, SLOT(blueAct()));
  connect(whiteAct, SIGNAL(triggered()), this, SLOT(whiteAct()));
  connect(blackAct, SIGNAL(triggered()), this, SLOT(blackAct()));
  connect(allColorsAct, SIGNAL(triggered()), this, SLOT(allColorsAct()));
  connect(saveColorCalibration, SIGNAL(triggered()), this, SLOT(saveColorCalibration()));

  QAction* colorButtons[] =
  {
    greenAct,
    yellowAct,
    orangeAct,
    redAct,
    blueAct,
    whiteAct,
    blackAct,
    allColorsAct
  };
  const int numOfColorButtons = sizeof(colorButtons) / sizeof(*colorButtons);

  menu->addAction(headAngleAct);
  menu->addAction(camUpAct);
  menu->addAction(camDownAct);

  menu->addSeparator();
  menu->addAction(headControlAct);

  menu->addSeparator();

  QActionGroup* colorGroup = new QActionGroup(menu);
  for(int i = 0; i < numOfColorButtons; ++i)
  {
    colorGroup->addAction(colorButtons[i]);
    colorButtons[i]->setCheckable(true);
    colorButtons[i]->setEnabled(imageView.segmented);
    menu->addAction(colorButtons[i]);
  }

  menu->addAction(saveColorCalibration);

  allColorsAct->setChecked(true);

  return menu;
}

ImageView::ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, bool upperCam, float gain) :
  upperCam(upperCam), fullName(fullName), icon(":/Icons/tag_green.png"), console(console),
  background(background), name(name),
  segmented(segmented),
  gain(gain), isActImage(strcmp(name.c_str(), "act") == 0) {}

SimRobot::Widget* ImageView::createWidget()
{
  return new ImageWidget(*this);
}
