/**
* @file Controller/Views/ImageView.h
*
* Declaration of class ImageView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#pragma once

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <QString>
#include <QIcon>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QSettings>
#include <QMenu>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "SimRobot.h"
#include "Tools/Math/Vector2.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"

class RobotConsole;

/**
* @class ImageView
*
* A class to represent a view displaying camera images and overlaid debug drawings.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/
class ImageView : public SimRobot::Object
{
public:
  /**
  * Constructor.
  * @param fullName The path to this view in the scene graph.
  * @param console The console object.
  * @param background The name of the background image.
  * @param name The name of the view.
  * @param segmented The image will be segmented.
  * @param gain The intensity is multiplied with this factor.
  */
  ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, bool upperCam, float gain = 1.0f);

  bool upperCam; /**< Show upper cams image in this view. */

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const std::string background; /**< The name of the background image. */
  const std::string name; /**< The name of the view. */
  bool segmented;  /**< The image will be segmented. */
  float gain; /**< The intensity is multiplied with this factor. */
  bool isActImage; /**< Whether this is an auto color table image view */

  /**
  * The method returns a new instance of a widget for this direct view.
  * The caller has to delete this instance. (Qt handles this)
  * @return The widget.
  */
  virtual SimRobot::Widget* createWidget();

  virtual const QString& getFullName() const {return fullName;}
  virtual const QIcon* getIcon() const {return &icon;}

  friend class ImageWidget;
};


class ImageWidget : public QWidget, public SimRobot::Widget
{
  Q_OBJECT
public:
  ImageWidget(ImageView& imageView);
  virtual ~ImageWidget();

private:
  ImageView& imageView;
  QImage* imageData;
  int imageWidth;
  int imageHeight;
  unsigned int lastImageTimeStamp;
  unsigned int lastColorReferenceTimeStamp;
  unsigned int lastDrawingsTimeStamp;
  QPainter painter;
  QPoint dragStart;
  QPoint dragPos;
  float zoom;
  QPoint offset;
  bool headControlMode;
  float imageXOffset;
  float imageYOffset;

  // which classified should be drawn?
  static const unsigned char allColors = ~0;
  unsigned char drawnColors;

  void updateColorCalibrator();
  void handleHeadControl(QMouseEvent& event);
  void paintEvent(QPaintEvent* event);
  virtual void paint(QPainter& painter);
  void paintDrawings(QPainter& painter);
  void copyImage(const Image& srcImage);
  void copyImageSegmented(const Image& srcImage);
  void paintImage(QPainter& painter, const Image& srcImage);
  bool needsRepaint() const;
  void window2viewport(QPoint& point);
  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void keyPressEvent(QKeyEvent* event);
  void wheelEvent(QWheelEvent* event);
  void mouseDoubleClickEvent(QMouseEvent* event);
  QSize sizeHint() const { return QSize(imageWidth, imageHeight); }
  virtual QWidget* getWidget() {return this;}
  virtual void update()
  {
    if(needsRepaint())
      QWidget::update();
  }

  virtual QMenu* createUserMenu() const;

  friend class ImageView;

private slots:

  void headControlToggled(bool value)
  {
    headControlMode = value;
    if(value)
    {
      imageView.console.handleConsole("mr HeadMotionRequest ManualHeadMotionProvider");
    }
  }
  void headAngle()
  {
    imageView.console.handleConsole("set representation:HeadAngleRequest pan = 1000; tilt = 1000; speed = 2.61799;");
  }
  void camUpOn()
  {
    imageView.console.handleConsole("ac " + imageView.name + " upper");
  }
  void camDownOn()
  {
    imageView.console.handleConsole("ac " + imageView.name + " lower");
  }

  void greenAct()
  {
    drawnColors = ColorReference::green;
    updateColorCalibrator();
  }
  void yellowAct()
  {
    drawnColors = ColorReference::yellow;
    updateColorCalibrator();
  }
  void orangeAct()
  {
    drawnColors = ColorReference::orange;
    updateColorCalibrator();
  }
  void redAct()
  {
    drawnColors = ColorReference::red;
    updateColorCalibrator();
  }
  void blueAct()
  {
    drawnColors = ColorReference::blue;
    updateColorCalibrator();
  }
  void whiteAct()
  {
    drawnColors = ColorReference::white;
    updateColorCalibrator();
  }
  void blackAct()
  {
    drawnColors = ColorReference::black;
    updateColorCalibrator();
  }
  void allColorsAct()
  {
    drawnColors = allColors;
    updateColorCalibrator();
  }
  void saveColorCalibration()
  {
    ColorReference& cr = imageView.console.colorReference;
    char buffer[1000];
    sprintf(buffer, 
    "set parameters:ColorProvider minHGreen = %f; maxHGreen = %f; minSGreen = %f; maxSGreen = %f; minVGreen = %f; maxVGreen = %f; minHYellow = %f; maxHYellow = %f; minSYellow = %f; maxSYellow = %f; minVYellow = %f; maxVYellow = %f; minHOrange = %f; maxHOrange = %f; minSOrange = %f; maxSOrange = %f; minVOrange = %f; maxVOrange = %f; minHRed = %f; maxHRed = %f; minSRed = %f; maxSRed = %f; minVRed = %f; maxVRed = %f; minHBlue = %f; maxHBlue = %f; minSBlue = %f; maxSBlue = %f; minVBlue = %f; maxVBlue = %f; minRWhite = %d; minBWhite = %d; minRBWhite = %d; cbBlack = 128; crBlack = 128; maxYBlack = 60;",
    cr.thresholdGreen.hue.min, cr.thresholdGreen.hue.max, cr.thresholdGreen.saturation.min, cr.thresholdGreen.saturation.max, cr.thresholdGreen.value.min, cr.thresholdGreen.value.max,
    cr.thresholdYellow.hue.min, cr.thresholdYellow.hue.max, cr.thresholdYellow.saturation.min, cr.thresholdYellow.saturation.max, cr.thresholdYellow.value.min, cr.thresholdYellow.value.max,
    cr.thresholdOrange.hue.min, cr.thresholdOrange.hue.max, cr.thresholdOrange.saturation.min, cr.thresholdOrange.saturation.max, cr.thresholdOrange.value.min, cr.thresholdOrange.value.max,
    cr.thresholdRed.hue.min, cr.thresholdRed.hue.max, cr.thresholdRed.saturation.min, cr.thresholdRed.saturation.max, cr.thresholdRed.value.min, cr.thresholdRed.value.max,
    cr.thresholdBlue.hue.min, cr.thresholdBlue.hue.max, cr.thresholdBlue.saturation.min, cr.thresholdBlue.saturation.max, cr.thresholdBlue.value.min, cr.thresholdBlue.value.max,
    cr.thresholdWhite.first, cr.thresholdWhite.second, cr.thresholdWhite.third
    );
    imageView.console.handleConsole(std::string(buffer));
    imageView.console.handleConsole("save parameters:ColorProvider");
    imageView.console.handleConsole("save representation:CameraSettings");
  }
};
