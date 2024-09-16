/**
 * @file SimulatedNao/Views/ImageView.cpp
 *
 * This file implements a view that displays an image overlaid by debug drawing.
 * Such a view is usually associated with a thread. It displays an image provided
 * by that thread. It prefers drawings originating from its associated thread. It
 * ignores drawings from other threads if the associated thread could also provide
 * them, but currently does not. It is also possible to create a view without an
 * image. Such a view can also be independent from a specific thread.
 *
 * @author Thomas Röfer
 * @author Colin Graf
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#include <QDir>
#include <QFileDialog>
#include <QMenu>
#include <QMouseEvent>
#include <QSettings>
#include <iostream>

#include "ImageView.h"
#include "SimulatedNao/RoboCupCtrl.h"
#include "SimulatedNao/RobotConsole.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "ImageProcessing/PixelTypes.h"

#include "Math/Eigen.h"

ImageView::ImageView(const QString& fullName, RobotConsole& console, const std::string& background,
                     const std::string& name, const std::string& threadName, float gain, float ddScale)
  : DrawingView(fullName, console, name, threadName, ":/Icons/icons8-image-50.png", 1.f),
    background(background), gain(gain), ddScale(ddScale) {}

SimRobot::Widget* ImageView::createWidget()
{
  return new ImageWidget(*this);
}

ImageWidget::ImageWidget(ImageView& view)
  : DrawingWidget(view, QPointF(-0.5f, -0.5f), view.console.imageViews[view.name])
{
  viewSize = QSize(CameraImage::maxResolutionWidth / 2, CameraImage::maxResolutionHeight / 2);
}

ImageWidget::~ImageWidget()
{
  delete imageData;
  if(imageDataStorage)
    Memory::alignedFree(imageDataStorage);
}

void ImageWidget::paint(QPainter& painter)
{
  SYNC_WITH(view.console);

  const DebugImage* image = nullptr;
  unsigned timestamp = 0;

  if(!view.threadName.empty())
  {
    RobotConsole::Images& currentImages = view.console.threadData[view.threadName].images;
    RobotConsole::Images::const_iterator i = currentImages.find(static_cast<ImageView&>(view).background);

    if(i != currentImages.end())
    {
      image = i->second.image;
      timestamp = i->second.timestamp;
      viewSize.setWidth(image->getImageWidth());
      viewSize.setHeight(image->height);
    }
    else if(!currentImages.empty())
    {
      viewSize.setWidth(currentImages.begin()->second.image->getImageWidth());
      viewSize.setHeight(currentImages.begin()->second.image->height);
    }
  }

  updateTransform(painter);

  if(image)
    paintImage(painter, *image, timestamp);
  else
    lastImageTimestamp = 0;

  painter.scale(static_cast<ImageView&>(view).ddScale, static_cast<ImageView&>(view).ddScale);
  paintDrawings(painter);
}

void ImageWidget::copyImage(const DebugImage& srcImage)
{
  int width = srcImage.width;
  int height = srcImage.height;

  const QImage::Format desiredFormat =  QImage::Format::Format_RGB32;

  if(!imageData || !imageDataStorage ||
     !(imageData->width() == srcImage.getImageWidth() && imageData->height() == height) ||
     imageData->format() != desiredFormat)
  {
    delete imageData;
    if(imageDataStorage)
      Memory::alignedFree(imageDataStorage);
    imageDataStorage = Memory::alignedMalloc(srcImage.getImageWidth() * height * 4 + 256, 32);
    imageData = new QImage(reinterpret_cast<unsigned char*>(imageDataStorage), srcImage.getImageWidth(), height, desiredFormat);
  }
  view.console.debugImageConverter.convertToBGRA(srcImage, imageData->scanLine(0));

  if(static_cast<ImageView&>(view).gain != 1.f)
  {
    unsigned* p = reinterpret_cast<unsigned*>(imageData->scanLine(0));
    float gain = static_cast<ImageView&>(view).gain;
    for(unsigned* pEnd = p + width * height; p < pEnd; ++p)
    {
      int r = static_cast<int>(gain * static_cast<float>((*p >> 16) & 0xff));
      int g = static_cast<int>(gain * static_cast<float>((*p >> 8) & 0xff));
      int b = static_cast<int>(gain * static_cast<float>((*p) & 0xff));

      *p = (r < 0 ? 0 : r > 255 ? 255 : r) << 16 |
           (g < 0 ? 0 : g > 255 ? 255 : g) << 8 |
           (b < 0 ? 0 : b > 255 ? 255 : b) |
           0xff000000;
    }
  }
}

void ImageWidget::paintImage(QPainter& painter, const DebugImage& srcImage, unsigned timestamp)
{
  if(timestamp != lastImageTimestamp)
  {
    if(srcImage.getImageWidth() > 1 && srcImage.height > 1)
      copyImage(srcImage);
    lastImageTimestamp = timestamp;
  }
  else if(!imageData || viewSize.width() != imageData->width() || viewSize.height() != imageData->height())
  {
    // make sure we have a buffer
    delete imageData;
    imageData = new QImage(viewSize, QImage::Format_RGB32);
  }

  painter.drawImage(QRectF(0, 0, viewSize.width(), viewSize.height()), *imageData);
}

bool ImageWidget::needsRepaint() const
{
  if(!view.threadName.empty())
  {
    SYNC_WITH(view.console);
    RobotConsole::Images& currentImages = view.console.threadData[view.threadName].images;
    RobotConsole::Images::const_iterator j = currentImages.find(static_cast<ImageView&>(view).background);
    if(j != currentImages.end())
      return j->second.timestamp != lastImageTimestamp;
  }

  return DrawingWidget::needsRepaint() || lastImageTimestamp != 0;
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
  const bool hovering = dragStart == QPointF(-1, -1);

  DrawingWidget::mouseMoveEvent(event);

  if(hovering && toolTip().isEmpty() && !view.threadName.empty())
  {
    QPointF pos(event->pos());
    window2viewport(pos);

    SYNC_WITH(view.console);
    RobotConsole::Images& currentImages = view.console.threadData[view.threadName].images;
    RobotConsole::Images::const_iterator i = currentImages.find(static_cast<ImageView&>(view).background);
    if(i != currentImages.end())
    {
      const DebugImage* image = i->second.image;
      if(pos.rx() >= 0 && pos.ry() >= 0 && pos.rx() < image->getImageWidth() && pos.ry() < image->height)
      {
        char tooltipstr[128];
        sprintf(tooltipstr, "x=%d, y=%d\n", static_cast<int>(pos.rx()), static_cast<int>(pos.ry()));

        char* tooltip = tooltipstr + strlen(tooltipstr);

        switch(image->type)
        {
          case PixelTypes::RGB:
          {
            const PixelTypes::RGBPixel& px = image->getView<PixelTypes::RGBPixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx())];
            sprintf(tooltip, "R=%d, G=%d, B=%d", px.r, px.g, px.b);
            break;
          }
          case PixelTypes::BGRA:
          {
            const PixelTypes::BGRAPixel& px = image->getView<PixelTypes::BGRAPixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx())];
            sprintf(tooltip, "R=%d, G=%d, B=%d", px.r, px.g, px.b);
            break;
          }
          case PixelTypes::YUV:
          {
            const PixelTypes::YUVPixel& px = image->getView<PixelTypes::YUVPixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx())];
            sprintf(tooltip, "Y=%d, U=%d, V=%d", px.y, px.u, px.v);
            break;
          }
          case PixelTypes::YUYV:
          {
            const PixelTypes::YUYVPixel& px = image->getView<PixelTypes::YUYVPixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx() / 2)];
            sprintf(tooltip, "Y=%d, U=%d, V=%d", px.y(static_cast<size_t>(pos.rx())), px.u, px.v);
            break;
          }
          case PixelTypes::Grayscale:
            sprintf(tooltip, "Luminosity=%d", image->getView<PixelTypes::GrayscaledPixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx())]);
            break;
          case PixelTypes::Hue:
            sprintf(tooltip, "Hue=%d", static_cast<int>(image->getView<PixelTypes::HuePixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx())]));
            break;
          case PixelTypes::Edge2:
          {
            const PixelTypes::Edge2Pixel& px = image->getView<PixelTypes::Edge2Pixel>()[static_cast<int>(pos.ry())][static_cast<int>(pos.rx())];
            sprintf(tooltip, "FilterX=%d, FilterY=%d", px.filterX, px.filterY);
            break;
          }
        }
        setToolTip(QString(tooltipstr));
      }
    }
  }
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
  QPointF pos = QPointF(event->pos());

  if(dragStart == pos || dragStart == QPointF(-1, -1))
  {
    window2viewport(pos);
    Vector2i v = Vector2i(static_cast<int>(pos.x()), static_cast<int>(pos.y()));

    if(event->modifiers() & Qt::AltModifier)
    {
      // update spot
      const char* action = nullptr;
      std::unordered_map<std::string, Pose2f> transforms;
      SYNC_WITH(view.console);
      for(const std::string& drawing : drawings)
        for(auto& [threadName, debugDrawing] : getDrawings(drawing))
        {
          auto transform = transforms.find(threadName);
          if(transform == transforms.end())
          {
            transforms.emplace(threadName, Pose2f());
            transform = transforms.find(threadName);
          }
          debugDrawing->updateOrigin(transform->second);
          action = debugDrawing->getSpot(static_cast<int>(pos.x()), static_cast<int>(pos.y()), transform->second);
          if(action)
          {
            view.console.handleConsole(action);
            goto actionDone;
          }
        }
    actionDone: ;
    }

    SYNC_WITH(view.console);
    const auto& commands = view.console.imageViewCommands[view.name];
    for(const auto& command : commands)
    {
      if((event->modifiers() & command.modifierMask) == command.modifiers)
      {
        std::stringstream ss;
        for(const auto& token : command.tokens)
        {
          switch(token.type)
          {
            case RobotConsole::ImageViewCommand::Token::literal:
              ss << token.string;
              break;
            case RobotConsole::ImageViewCommand::Token::placeholder:
              if(token.id == 1)
                ss << v.x();
              else if(token.id == 2)
                ss << v.y();
              else if(token.id == 3)
                ss << (static_cast<char>(view.threadName[0] | 0x20) + view.threadName.substr(1));
              break;
          }
        }
        view.console.handleConsole(ss.str());
      }
    }
  }

  DrawingWidget::mouseReleaseEvent(event);
}

std::vector<std::pair<std::string, const DebugDrawing*>> ImageWidget::getDrawings(const std::string& name) const
{
  return DrawingWidget::getDrawings(name, [](const RobotConsole::ThreadData& data) -> const RobotConsole::Drawings&
  {
    return data.imageDrawings;
  });
}

QMenu* ImageWidget::createUserMenu() const
{
  QMenu* menu = new QMenu(tr("&Image"));

  menu->addSeparator();

  QAction* saveImgAct = new QAction(tr("&Save Image"), menu);
  connect(saveImgAct, &QAction::triggered, this, &ImageWidget::saveImg);
  menu->addAction(saveImgAct);

  return menu;
}

void ImageWidget::saveImg()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save as PNG"), settings.value("ExportDirectory", "").toString(), tr("(*.png)")
#ifdef LINUX
                                                  , nullptr, QFileDialog::DontUseNativeDialog
#endif
                                                  );
  if(fileName.isEmpty())
    return;
  if(!QFileInfo(fileName).fileName().contains("."))
  {
    // Enforce .png ending
    fileName += ".png";
  }
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  SYNC_WITH(view.console);

  const DebugImage* image = nullptr;
  unsigned timestamp = 0;
  if(!view.threadName.empty())
  {
    RobotConsole::Images& currentImages = view.console.threadData[view.threadName].images;
    RobotConsole::Images::const_iterator i = currentImages.find(static_cast<ImageView&>(view).background);
    if(i != currentImages.end())
    {
      image = i->second.image;
      timestamp = i->second.timestamp;
      viewSize.setWidth(image->type == PixelTypes::YUYV ? image->width * 2 : image->width);
      viewSize.setHeight(image->height);
    }
  }

  QPixmap pixmap(!image ? viewSize.width() : image->type == PixelTypes::YUYV
                 ? image->width * 2 : image->width, !image ? viewSize.height() : image->height);
  QPainter painter(&pixmap);
  if(image)
    paintImage(painter, *image, timestamp);
  painter.scale(static_cast<ImageView&>(view).ddScale, static_cast<ImageView&>(view).ddScale);
  paintDrawings(painter);
  pixmap.save(fileName, "PNG");
}
