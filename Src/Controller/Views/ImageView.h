/**
 * @file Controller/Views/ImageView.h
 *
 * Declaration of class ImageView
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include <QString>
#include <QIcon>
#include <QImage>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QSettings>
#include <QMenu>
#include <QElapsedTimer>

#include <SimRobot.h>
#include "Controller/RoboCupCtrl.h"
#include "Controller/RobotConsole.h"
#include "Controller/Views/ColorCalibrationView/ColorCalibrationView.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Tools/Math/Eigen.h"

class RobotConsole;
class ImageWidget;

/**
 * @class ImageView
 *
 * A class to represent a view displaying camera images and overlaid debug drawings.
 *
 * @author Thomas Röfer
 */
class ImageView : public SimRobot::Object
{
public:
  const std::string threadIdentifier; /**< The thread that created the images shown in this view. */
  ImageWidget* widget = nullptr; /**< The widget of this view */

  /**
   * @param fullName The path to this view in the scene graph.
   * @param console The console object.
   * @param background The name of the background image.
   * @param name The name of the view.
   * @param segmented The image will be segmented.
   * @param gain The intensity is multiplied with this factor.
   * @param ddScale The debug drawings are multiplied with this factor.
   */
  ImageView(const QString& fullName, RobotConsole& console, const std::string& background, const std::string& name, bool segmented, const std::string& threadIdentifier, float gain = 1.0f, float ddScale = 1.0f);

private:
  const QString fullName; /**< The path to this view in the scene graph */
  const QIcon icon; /**< The icon used for listing this view in the scene graph */
  RobotConsole& console; /**< A reference to the console object. */
  const std::string background; /**< The name of the background image. */
  const std::string name; /**< The name of the view. */
  bool segmented;  /**< The image will be segmented. */
  float gain; /**< The intensity is multiplied with this factor. */
  float ddScale; /**< The debug drawings are multiplied with this factor. */

  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

  const QString& getFullName() const override { return fullName; }
  const QIcon* getIcon() const override { return &icon; }

  friend class ImageWidget;
};

class ImageWidget : public WIDGET2D, public SimRobot::Widget
{
  Q_OBJECT
public:
  ImageWidget(ImageView& imageView);
  ~ImageWidget();

private:
  ImageView& imageView;
  QImage* imageData = nullptr;
  void* imageDataStorage = nullptr;
  int imageWidth = CameraImage::maxResolutionWidth;
  int imageHeight = CameraImage::maxResolutionHeight;
  unsigned int lastImageTimestamp = 0;
  unsigned int lastColorTableTimestamp = 0;
  unsigned int lastDrawingsTimestamp = 0;
  QPainter painter;
  QPointF dragStart;
  QPointF dragStartOffset;
  QPointF mousePos;
  float zoom = 1.f;
  float scale = 1.f;
  QPointF offset;

  void paintEvent(QPaintEvent* event) override;
  void paint(QPainter& painter) override;
  void paintDrawings(QPainter& painter);
  void copyImage(const DebugImage& srcImage);
  void copyImageSegmented(const DebugImage& srcImage);
  void segmentImage(const DebugImage& srcImage);
  void paintImage(QPainter& painter, const DebugImage& srcImage);
  bool needsRepaint() const;
  void window2viewport(QPointF& point);
  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  bool event(QEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  const DebugDrawing* getDrawing(const std::string& name) const;

  QSize sizeHint() const override { return QSize(imageWidth, imageHeight); }

  QWidget* getWidget() override { return this; }

  void update() override
  {
    if(needsRepaint())
      QWidget::update();
  }

  QMenu* createUserMenu() const override;

  friend class ImageView;

private slots:
  void saveImg();
};
