/**
 * @file Controller/Views/ImageView.cpp
 *
 * Implementation of class ImageView
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author Colin Graf
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QPinchGesture>
#include <QTouchEvent>
#include <QWidget>
#include <QSettings>
#include <QSignalMapper>
#include <QMenu>
#include <QProcess>
#include <QTemporaryFile>
#include <QMessageBox>
#include <sstream>
#include <iostream>

#include "ImageView.h"
#include "ColorCalibrationView/ColorCalibrationView.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/ImageProcessing/YHSColorConversion.h"
#include "Tools/Math/Approx.h"
#include <QFileDialog>

#include "Tools/Math/Eigen.h"

ImageView::ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, bool upperCam, float gain) :
  upperCam(upperCam), fullName(fullName), icon(":/Icons/tag_green.png"), console(console),
  background(background), name(name), segmented(segmented),
  gain(gain), isActImage(strcmp(name.c_str(), "act") == 0)
{}

void ImageView::forwardLastImage()
{
  if(widget != nullptr)
    widget->forwardLastImage();
}

SimRobot::Widget* ImageView::createWidget()
{
  widget = new ImageWidget(*this);
  return widget;
}

ImageWidget::ImageWidget(ImageView& imageView) :
  imageView(imageView), dragStart(-1, -1), offset(0, 0)
{
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  grabGesture(Qt::PinchGesture);
  setAttribute(Qt::WA_AcceptTouchEvents);

  QSettings& settings = RoboCupCtrl::application->getLayoutSettings();
  settings.beginGroup(imageView.fullName);
  zoom = (float)settings.value("Zoom", 1.).toDouble();
  offset = settings.value("Offset", QPoint()).toPoint();
  drawnColor = (FieldColors::Color) settings.value("DrawnColor", 1).toInt();
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

  imageView.widget = nullptr;

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

  const DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);

  if(i != currentImages.end())
  {
    image = i->second.image;
    imageWidth = image->getImageWidth();
    imageHeight = image->height;
  }
  else if(!currentImages.empty())
  {
    imageWidth = currentImages.begin()->second.image->getImageWidth();
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

void ImageWidget::copyImage(const DebugImage& srcImage)
{
  int width = srcImage.width;
  int height = srcImage.height;

  const QImage::Format desiredFormat =  QImage::Format::Format_RGB32;

  if(!imageData ||
     !(imageData->width() == srcImage.getImageWidth() && imageData->height() == height) ||
     imageData->format() != desiredFormat)
  {
    if(imageData)
      delete imageData;
    imageData = new QImage(srcImage.getImageWidth(), height, desiredFormat);
  }

  void* dest = imageData->bits();
  if(srcImage.type == PixelTypes::BGRA)
  {
    memcpy(dest, srcImage.getView<PixelTypes::BGRAPixel>()[0], width * height * PixelTypes::pixelSize(srcImage.type));
  }
  else
  {
    srcImage.convertToBGRA(dest);
  }

  if(imageView.gain != 1.f)
  {
    unsigned* p = (unsigned*)imageData->bits();
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

void ImageWidget::copyImageSegmented(const DebugImage& srcImage)
{
  if(!imageData ||
     !(imageData->width() == srcImage.getImageWidth() && imageData->height() == srcImage.height) ||
     imageData->format() != QImage::Format::Format_RGB32)
  {
    if(imageData)
      delete imageData;
    imageData = new QImage(srcImage.getImageWidth(), srcImage.height, QImage::Format::Format_RGB32);
  }

  if(_supportsAVX2)
    segmentImage<true>(srcImage);
  else
    segmentImage<false>(srcImage);
}

template<bool avx> void ImageWidget::segmentImage(const DebugImage& srcImage)
{
  static const unsigned int baseColorsBGRA[FieldColors::numOfColors] =
  {
    0xff7f7f7f,
    0xffffffff, //white
    0xff000000, //black
    0xff00ff00, //green
    0xff0000ff, //own team color
    0xffff0000 //opponent team color
  };
  const __m_auto_i noneBGRA = _mmauto_set1_epi32(baseColorsBGRA[FieldColors::none]);
  const __m_auto_i whiteBGRA = drawAllColors || drawnColor == FieldColors::white ? _mmauto_set1_epi32(baseColorsBGRA[FieldColors::white]) : noneBGRA;
  const __m_auto_i blackBGRA = drawAllColors || drawnColor == FieldColors::black ? _mmauto_set1_epi32(baseColorsBGRA[FieldColors::black]) : noneBGRA;
  alignas(avx ? 32 : 16) __m_auto_i colorsBGRA[FieldColors::numOfColors - FieldColors::numOfNonColors];
  for(FieldColors::Color i = FieldColors::numOfNonColors; i < FieldColors::numOfColors; i = (FieldColors::Color)((unsigned char)(i) + 1))
  {
    _mmauto_storet_si_all<true>(colorsBGRA + (i - FieldColors::numOfNonColors), drawAllColors || drawnColor == i ? _mmauto_set1_epi32(baseColorsBGRA[i]) : noneBGRA);
  }

  const FieldColors& theFieldColors = imageView.console.colorCalibration;
  const __m_auto_i blackYTo = _mmauto_set1_epi8(theFieldColors.maxYBlack);
  const __m_auto_i whiteYFrom = _mmauto_set1_epi8(theFieldColors.minYWhite);
  alignas(avx ? 32 : 16) __m_auto_i fieldColorYFrom[FieldColors::numOfColors - FieldColors::numOfNonColors];
  alignas(avx ? 32 : 16) __m_auto_i fieldColorYTo[FieldColors::numOfColors - FieldColors::numOfNonColors];
  alignas(avx ? 32 : 16) __m_auto_i fieldColorHFrom[FieldColors::numOfColors - FieldColors::numOfNonColors];
  alignas(avx ? 32 : 16) __m_auto_i fieldColorHTo[FieldColors::numOfColors - FieldColors::numOfNonColors];
  alignas(avx ? 32 : 16) __m_auto_i fieldColorSFrom[FieldColors::numOfColors - FieldColors::numOfNonColors];
  alignas(avx ? 32 : 16) __m_auto_i fieldColorSTo[FieldColors::numOfColors - FieldColors::numOfNonColors];
  for(FieldColors::Color i = FieldColors::numOfNonColors; i < FieldColors::numOfColors; i = (FieldColors::Color)((unsigned char)(i) + 1))
  {
    _mmauto_storet_si_all<true>(&fieldColorYFrom[i - FieldColors::numOfNonColors], _mmauto_set1_epi8(theFieldColors[i].y.min));
    _mmauto_storet_si_all<true>(&fieldColorYTo[i - FieldColors::numOfNonColors], _mmauto_set1_epi8(theFieldColors[i].y.max));
    _mmauto_storet_si_all<true>(&fieldColorHFrom[i - FieldColors::numOfNonColors], _mmauto_set1_epi8(theFieldColors[i].h.min));
    _mmauto_storet_si_all<true>(&fieldColorHTo[i - FieldColors::numOfNonColors], _mmauto_set1_epi8(theFieldColors[i].h.max));
    _mmauto_storet_si_all<true>(&fieldColorSFrom[i - FieldColors::numOfNonColors], _mmauto_set1_epi8(theFieldColors[i].s.min));
    _mmauto_storet_si_all<true>(&fieldColorSTo[i - FieldColors::numOfNonColors], _mmauto_set1_epi8(theFieldColors[i].s.max));
  }

  static const __m_auto_i c_0 = _mmauto_setzero_si_all();
  static const __m_auto_i c_128 = _mmauto_set1_epi8(char(128));
  static const __m_auto_i channelMask = _mmauto_set1_epi16(0x00FF);

  const __m_auto_i* const srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage.getView<PixelTypes::YUYVPixel>()[srcImage.height]);
  __m_auto_i* dest = reinterpret_cast<__m_auto_i*>(imageData->bits());
  alignas(avx ? 32 : 16) __m_auto_i buffer[8];
  for(const __m_auto_i* src = reinterpret_cast<const __m_auto_i*>(srcImage.getView<PixelTypes::YUYVPixel>()[0]); src < srcEnd; dest += 8)
  {
    memset(buffer, 0, sizeof(buffer));
    const __m_auto_i p0 = _mmauto_loadt_si_all<true>(src++);
    const __m_auto_i p1 = _mmauto_loadt_si_all<true>(src++);
    const __m_auto_i p2 = _mmauto_loadt_si_all<true>(src++);
    const __m_auto_i p3 = _mmauto_loadt_si_all<true>(src++);

    // Compute luminance
    const __m_auto_i y0 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p0, channelMask), _mmauto_and_si_all(p1, channelMask)));
    const __m_auto_i y1 = _mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(p2, channelMask), _mmauto_and_si_all(p3, channelMask)));

    // Compute saturation
    const __m_auto_i uv0 = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p0, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p1, 1), channelMask))), c_128);
    const __m_auto_i uv1 = _mmauto_sub_epi8(_mmauto_correct_256op(_mmauto_packus_epi16(_mmauto_and_si_all(_mmauto_srli_si_all(p2, 1), channelMask), _mmauto_and_si_all(_mmauto_srli_si_all(p3, 1), channelMask))), c_128);
    const __m_auto_i sat = YHSColorConversion::computeSaturation<avx>(uv0, uv1);

    // Compute hue
    const __m_auto_i hue = YHSColorConversion::computeHue<avx>(uv0, uv1);

    // Classify colors
    __m_auto_i bufVal;
    for(FieldColors::Color i = (FieldColors::Color)0; i < FieldColors::numOfColors - FieldColors::numOfNonColors; i = (FieldColors::Color)((unsigned char)(i) + 1))
    {
      const __m_auto_i isHSNotOfColor = _mmauto_or_si_all(
                                          _mmauto_or_si_all(_mmauto_subs_epu8(_mmauto_loadt_si_all<true>(&fieldColorHFrom[i]), hue), _mmauto_subs_epu8(hue, _mmauto_loadt_si_all<true>(&fieldColorHTo[i]))),
                                          _mmauto_or_si_all(_mmauto_subs_epu8(_mmauto_loadt_si_all<true>(&fieldColorSFrom[i]), sat), _mmauto_subs_epu8(sat, _mmauto_loadt_si_all<true>(&fieldColorSTo[i])))
                                        );
      __m_auto_i isHS0NotOfColor = isHSNotOfColor;
      __m_auto_i isHS1NotOfColor = isHSNotOfColor;
      _mmauto_unpacklohi_epi8(isHS0NotOfColor, isHS1NotOfColor);

      const __m_auto_i colorBGRA = _mmauto_loadt_si_all<true>(colorsBGRA + i);

      __m_auto_i isOfColor = _mmauto_cmpeq_epi8(
                               _mmauto_or_si_all(
                                 _mmauto_or_si_all(_mmauto_subs_epu8(_mmauto_loadt_si_all<true>(&fieldColorYFrom[i]), y0), _mmauto_subs_epu8(y0, _mmauto_loadt_si_all<true>(&fieldColorYTo[i]))),
                                 isHS0NotOfColor
                               ),
                               c_0
                             );
      __m_auto_i isOfColor0 = isOfColor;
      __m_auto_i isOfColor2 = isOfColor;
      _mmauto_unpacklohi_epi8(isOfColor0, isOfColor2);
      __m_auto_i isOfColor1 = isOfColor0;
      _mmauto_unpacklohi_epi8(isOfColor0, isOfColor1);
      __m_auto_i isOfColor3 = isOfColor2;
      _mmauto_unpacklohi_epi8(isOfColor2, isOfColor3);

      bufVal = _mmauto_loadt_si_all<true>(buffer);
      _mmauto_storet_si_all<true>(buffer, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor0, colorBGRA))));
      bufVal = _mmauto_loadt_si_all<true>(buffer + 1);
      _mmauto_storet_si_all<true>(buffer + 1, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor1, colorBGRA))));
      bufVal = _mmauto_loadt_si_all<true>(buffer + 2);
      _mmauto_storet_si_all<true>(buffer + 2, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor2, colorBGRA))));
      bufVal = _mmauto_loadt_si_all<true>(buffer + 3);
      _mmauto_storet_si_all<true>(buffer + 3, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor3, colorBGRA))));

      isOfColor = _mmauto_cmpeq_epi8(
                    _mmauto_or_si_all(
                      _mmauto_or_si_all(_mmauto_subs_epu8(_mmauto_loadt_si_all<true>(&fieldColorYFrom[i]), y1), _mmauto_subs_epu8(y1, _mmauto_loadt_si_all<true>(&fieldColorYTo[i]))),
                      isHS1NotOfColor
                    ),
                    c_0
                  );
      isOfColor0 = isOfColor;
      isOfColor2 = isOfColor;
      _mmauto_unpacklohi_epi8(isOfColor0, isOfColor2);
      isOfColor1 = isOfColor0;
      _mmauto_unpacklohi_epi16(isOfColor0, isOfColor1);
      isOfColor3 = isOfColor2;
      _mmauto_unpacklohi_epi16(isOfColor2, isOfColor3);

      bufVal = _mmauto_loadt_si_all<true>(buffer + 4);
      _mmauto_storet_si_all<true>(buffer + 4, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor0, colorBGRA))));
      bufVal = _mmauto_loadt_si_all<true>(buffer + 5);
      _mmauto_storet_si_all<true>(buffer + 5, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor1, colorBGRA))));
      bufVal = _mmauto_loadt_si_all<true>(buffer + 6);
      _mmauto_storet_si_all<true>(buffer + 6, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor2, colorBGRA))));
      bufVal = _mmauto_loadt_si_all<true>(buffer + 7);
      _mmauto_storet_si_all<true>(buffer + 7, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(_mmauto_or_si_all(_mmauto_cmpeq_epi32(bufVal, c_0), _mmauto_cmpeq_epi32(bufVal, noneBGRA)), _mmauto_and_si_all(isOfColor3, colorBGRA))));
    }

    // Classify non-colors
    __m_auto_i isWhite = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(whiteYFrom, y0), c_0);
    __m_auto_i isWhite0 = isWhite;
    __m_auto_i isWhite2 = isWhite;
    _mmauto_unpacklohi_epi8(isWhite0, isWhite2);
    __m_auto_i isWhite1 = isWhite0;
    _mmauto_unpacklohi_epi16(isWhite0, isWhite1);
    __m_auto_i isWhite3 = isWhite2;
    _mmauto_unpacklohi_epi16(isWhite2, isWhite3);
    __m_auto_i isBlack = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(y0, blackYTo), c_0);
    __m_auto_i isBlack0 = isBlack;
    __m_auto_i isBlack2 = isBlack;
    _mmauto_unpacklohi_epi8(isBlack0, isBlack2);
    __m_auto_i isBlack1 = isBlack0;
    _mmauto_unpacklohi_epi16(isBlack0, isBlack1);
    __m_auto_i isBlack3 = isBlack2;
    _mmauto_unpacklohi_epi16(isBlack2, isBlack3);

    bufVal = _mmauto_loadt_si_all<true>(buffer);
    _mmauto_storet_si_all<true>(buffer, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite0, whiteBGRA),
                                      _mmauto_and_si_all(isBlack0, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite0, isBlack0), noneBGRA)
                                  )
                                )));
    bufVal = _mmauto_loadt_si_all<true>(buffer + 1);
    _mmauto_storet_si_all<true>(buffer + 1, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite1, whiteBGRA),
                                      _mmauto_and_si_all(isBlack1, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite1, isBlack1), noneBGRA)
                                  )
                                )));
    bufVal = _mmauto_loadt_si_all<true>(buffer + 2);
    _mmauto_storet_si_all<true>(buffer + 2, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite2, whiteBGRA),
                                      _mmauto_and_si_all(isBlack2, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite2, isBlack2), noneBGRA)
                                  )
                                )));
    bufVal = _mmauto_loadt_si_all<true>(buffer + 3);
    _mmauto_storet_si_all<true>(buffer + 3, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite3, whiteBGRA),
                                      _mmauto_and_si_all(isBlack3, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite3, isBlack3), noneBGRA)
                                  )
                                )));

    isWhite = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(whiteYFrom, y1), c_0);
    isWhite0 = isWhite;
    isWhite2 = isWhite;
    _mmauto_unpacklohi_epi8(isWhite0, isWhite2);
    isWhite1 = isWhite0;
    _mmauto_unpacklohi_epi8(isWhite0, isWhite1);
    isWhite3 = isWhite2;
    _mmauto_unpacklohi_epi8(isWhite2, isWhite3);
    isBlack = _mmauto_cmpeq_epi8(_mmauto_subs_epu8(y1, blackYTo), c_0);
    isBlack0 = isBlack;
    isBlack2 = isBlack;
    _mmauto_unpacklohi_epi8(isBlack0, isBlack2);
    isBlack1 = isBlack0;
    _mmauto_unpacklohi_epi8(isBlack0, isBlack1);
    isBlack3 = isBlack2;
    _mmauto_unpacklohi_epi8(isBlack2, isBlack3);

    bufVal = _mmauto_loadt_si_all<true>(buffer + 4);
    _mmauto_storet_si_all<true>(buffer + 4, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite0, whiteBGRA),
                                      _mmauto_and_si_all(isBlack0, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite0, isBlack0), noneBGRA)
                                  )
                                )));
    bufVal = _mmauto_loadt_si_all<true>(buffer + 5);
    _mmauto_storet_si_all<true>(buffer + 5, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite1, whiteBGRA),
                                      _mmauto_and_si_all(isBlack1, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite1, isBlack1), noneBGRA)
                                  )
                                )));
    bufVal = _mmauto_loadt_si_all<true>(buffer + 6);
    _mmauto_storet_si_all<true>(buffer + 6, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite2, whiteBGRA),
                                      _mmauto_and_si_all(isBlack2, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite2, isBlack2), noneBGRA)
                                  )
                                )));
    bufVal = _mmauto_loadt_si_all<true>(buffer + 7);
    _mmauto_storet_si_all<true>(buffer + 7, _mmauto_or_si_all(bufVal, _mmauto_and_si_all(
                                  _mmauto_cmpeq_epi32(bufVal, c_0),
                                  _mmauto_or_si_all(
                                    _mmauto_or_si_all(
                                      _mmauto_and_si_all(isWhite3, whiteBGRA),
                                      _mmauto_and_si_all(isBlack3, blackBGRA)
                                    ),
                                    _mmauto_andnot_si_all(_mmauto_or_si_all(isWhite3, isBlack3), noneBGRA)
                                  )
                                )));

    memcpy(dest, buffer, sizeof(buffer));
  }
}

void ImageWidget::paintImage(QPainter& painter, const DebugImage& srcImage)
{
  if(srcImage.timeStamp != lastImageTimeStamp || imageView.segmented)
  {
    if(imageView.segmented)
      copyImageSegmented(srcImage);
    else
      copyImage(srcImage);

    lastImageTimeStamp = srcImage.timeStamp;
    if(imageView.segmented)
      lastColorTableTimeStamp = imageView.console.colorCalibrationTimeStamp;
  }
  else if(!imageData || imageWidth != imageData->width() || imageHeight != imageData->height())
  {
    // make sure we have a buffer
    if(imageData)
      delete imageData;
    imageData = new QImage(imageWidth, imageHeight, QImage::Format_RGB32);
  }

  painter.drawImage(QRectF(0, 0, imageWidth, imageHeight), *imageData);
}

bool ImageWidget::needsRepaint() const
{
  SYNC_WITH(imageView.console);
  DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator j = currentImages.find(imageView.background);
  if(j != currentImages.end())
    image = j->second.image;

  if(!image)
  {
    const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
    for(const std::string& drawing : drawings)
    {
      const DebugDrawing& debugDrawing(imageView.upperCam ? imageView.console.upperCamImageDrawings[drawing] : imageView.console.lowerCamImageDrawings[drawing]);
      if(debugDrawing.timeStamp > lastDrawingsTimeStamp)
        return true;
    }
    return lastImageTimeStamp != 0;
  }
  else
    return image->timeStamp != lastImageTimeStamp ||
           (imageView.segmented && imageView.console.colorCalibrationTimeStamp != lastColorTableTimeStamp);
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

  DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);
  if(i != currentImages.end())
    image = i->second.image;

  // Update tool tip
  const char* text = 0;
  const std::list<std::string>& drawings(imageView.console.imageViews[imageView.name]);
  Pose2f origin;
  for(const std::string& drawing : drawings)
  {
    const DebugDrawing& debugDrawing(imageView.upperCam ? imageView.console.upperCamImageDrawings[drawing] : imageView.console.lowerCamImageDrawings[drawing]);
    debugDrawing.updateOrigin(origin);
    text = debugDrawing.getTip(pos.rx(), pos.ry(), origin);
    if(text)
      break;
  }

  if(text)
    setToolTip(QString(text));
  else if(image && pos.rx() >= 0 && pos.ry() >= 0 && pos.rx() < image->getImageWidth() && pos.ry() < image->height)
  {
    unsigned char y, u, v, r, g, b, h, s, i;
    switch(image->type)
    {
      case PixelTypes::RGB:
        r = image->getView<PixelTypes::RGBPixel>()[pos.ry()][pos.rx()].r;
        g = image->getView<PixelTypes::RGBPixel>()[pos.ry()][pos.rx()].g;
        b = image->getView<PixelTypes::RGBPixel>()[pos.ry()][pos.rx()].b;
        ColorModelConversions::fromRGBToHSI(r, g, b, h, s, i);
        ColorModelConversions::fromRGBToYUV(r, g, b, y, u, v);
        break;
      case PixelTypes::BGRA:
        r = image->getView<PixelTypes::BGRAPixel>()[pos.ry()][pos.rx()].r;
        g = image->getView<PixelTypes::BGRAPixel>()[pos.ry()][pos.rx()].g;
        b = image->getView<PixelTypes::BGRAPixel>()[pos.ry()][pos.rx()].b;
        ColorModelConversions::fromRGBToHSI(r, g, b, h, s, i);
        ColorModelConversions::fromRGBToYUV(r, g, b, y, u, v);
        break;
      case PixelTypes::YUV:
        y = image->getView<PixelTypes::YUVPixel>()[pos.ry()][pos.rx()].y;
        u = image->getView<PixelTypes::YUVPixel>()[pos.ry()][pos.rx()].u;
        v = image->getView<PixelTypes::YUVPixel>()[pos.ry()][pos.rx()].v;
        ColorModelConversions::fromYUVToRGB(y, u, v, r, g, b);
        ColorModelConversions::fromRGBToHSI(r, g, b, h, s, i);
        break;
      case PixelTypes::YUYV:
        y = image->getView<PixelTypes::YUYVPixel>()[pos.ry()][pos.rx() / 2].y(pos.rx());
        u = image->getView<PixelTypes::YUYVPixel>()[pos.ry()][pos.rx() / 2].u;
        v = image->getView<PixelTypes::YUYVPixel>()[pos.ry()][pos.rx() / 2].v;
        ColorModelConversions::fromYUVToRGB(y, u, v, r, g, b);
        ColorModelConversions::fromRGBToHSI(r, g, b, h, s, i);
        break;
      case PixelTypes::Grayscale:
        y = image->getView<PixelTypes::GrayscaledPixel>()[pos.ry()][pos.rx()];
        u = v = 128;
        r = g = b = y;
        ColorModelConversions::fromRGBToHSI(r, g, b, h, s, i);
        break;
      case PixelTypes::Colored:
        switch(image->getView<PixelTypes::ColoredPixel>()[pos.ry()][pos.rx()])
        {
          case FieldColors::none:
            r = g = b = y = u = v = 128;
            break;
          case FieldColors::white:
            r = g = b = y = 255;
            u = v = 128;
            break;
          case FieldColors::black:
            r = g = b = y = 0;
            u = v = 128;
            break;
          case FieldColors::green:
            r = b = 0;
            g = 255;
            ColorModelConversions::fromRGBToYUV(r, g, b, y, u, v);
            break;
          default:
            ASSERT(false);
            return;
        }
        ColorModelConversions::fromRGBToHSI(r, g, b, h, s, i);
        break;
      default:
        ASSERT(false);
        return;
    }
    char color[128];

    sprintf(color, "x=%d, y=%d\ny=%d, cb=%d, cr=%d\nr=%d, g=%d, b=%d\nh=%d, s=%d, i=%d", pos.rx(), pos.ry(), y, u, v, r, g, b, h, s, i);
    setToolTip(QString(color));
  }
  else
    setToolTip(QString());
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
        DebugImage* image = nullptr;
        RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
        RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);
        if(i != currentImages.end())
          image = i->second.image;
        if(image && pos.x() >= 0 && pos.y() >= 0 && pos.x() < image->width && pos.y() < image->height)
          imageView.console.colorCalibrationView->widget->expandCurrentColor(image->getView<PixelTypes::YUYVPixel>()[pos.y()][pos.x()], event->modifiers() & Qt::ControlModifier);
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
#ifdef FIX_MACOS_NO_CENTER_IN_PINCH_GESTURE_BUG
      QPoint center = mapFromGlobal(QCursor::pos());
#else
      QPoint center(static_cast<int>(pinch->centerPoint().x()),
                    static_cast<int>(pinch->centerPoint().y()));
#endif
      QPoint before(center);
      window2viewport(before);
      scale /= zoom;
#ifdef FIX_MACOS_PINCH_SCALE_RELATIVE_BUG
      pinch->setLastScaleFactor(1.f);
#endif
      zoom *= pinch->scaleFactor() / pinch->lastScaleFactor();
      if(zoom > 3.f)
        zoom = 3.f;
      else if(zoom < 0.1f)
        zoom = 0.1f;
      scale *= zoom;
      QPoint after(center);
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
#ifndef MACOS
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

  QAction* colorButtons[FieldColors::numOfColors - 1] =
  {
    new QAction(QIcon(":/Icons/white.png"), tr("Show Only &White"), menu),
    new QAction(QIcon(":/Icons/black.png"), tr("Show Only &Black"), menu),
    new QAction(QIcon(":/Icons/green.png"), tr("Show Only &Green"), menu),
    new QAction(QIcon(":/Icons/ownColor.png"), tr("Show Only &Own Jersey Color"), menu),
    new QAction(QIcon(":/Icons/opponentColor.png"), tr("Show Only O&pponent Jersey Color"), menu)
  };

  QActionGroup* colorGroup = new QActionGroup(menu);
  QSignalMapper* signalMapper = new QSignalMapper(const_cast<ImageWidget*>(this));
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(colorAct(int)));
  for(int i = 0; i < FieldColors::numOfColors - 1; ++i)
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

void ImageWidget::forwardLastImage()
{
  DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator j = currentImages.find(imageView.background);
  if(j != currentImages.end())
    image = j->second.image;
}

void ImageWidget::saveImg()
{
  QSettings& settings = RoboCupCtrl::application->getSettings();
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save as PNG"), settings.value("ExportDirectory", "").toString(), tr("(*.png)"));
  if(fileName.isEmpty())
    return;
  if(!QFileInfo(fileName).fileName().contains("."))
  {
    // Enforce .png ending
    fileName += ".png";
  }
  settings.setValue("ExportDirectory", QFileInfo(fileName).dir().path());

  SYNC_WITH(imageView.console);

  const DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = imageView.upperCam ? imageView.console.upperCamImages : imageView.console.lowerCamImages;
  RobotConsole::Images::const_iterator i = currentImages.find(imageView.background);

  if(i != currentImages.end())
  {
    image = i->second.image;
    imageWidth = image->type == PixelTypes::YUYV ? image->width * 2 : image->width;
    imageHeight = image->height;
  }
  if(image)
  {
    QPixmap pixmap(image->type == PixelTypes::YUYV ? image->width * 2 : image->width, image->height);
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
      view.second->widget->setDrawnColor((FieldColors::Color) color);

  ColorCalibrationView* colorView = imageView.console.colorCalibrationView;
  if(colorView && colorView->widget)
  {
    colorView->widget->updateWidgets((FieldColors::Color) color);
    colorView->widget->currentCalibrationChanged();
  }
}

void ImageWidget::setUndoRedo(const bool enableUndo, const bool enableRedo)
{
  if(undoAction && redoAction)
  {
    ColorCalibrationView* colorView = imageView.console.colorCalibrationView;
    disconnect(undoAction, nullptr, nullptr, nullptr);
    disconnect(redoAction, nullptr, nullptr, nullptr);
    connect(undoAction, SIGNAL(triggered()), colorView->widget, SLOT(undoColorCalibration()));
    connect(redoAction, SIGNAL(triggered()), colorView->widget, SLOT(redoColorCalibration()));
    undoAction->setEnabled(enableUndo);
    redoAction->setEnabled(enableRedo);
  }
}
