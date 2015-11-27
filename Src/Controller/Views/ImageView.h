/**
* @file Controller/Views/ImageView.h
*
* Declaration of class ImageView
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author Colin Graf
*/

#pragma once

#include <QString>
#include <QIcon>
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <QWidget>
#include <QSettings>
#include <QMenu>

#include <SimRobot.h>
#include "Tools/Math/Eigen.h"
#include "Controller/RobotConsole.h"
#include "Controller/RoboCupCtrl.h"
#include "Controller/Views/ColorCalibrationView/ColorCalibrationView.h"
#include "Controller/Visualization/PaintMethods.h"
#include "Controller/ImageViewAdapter.h"
#include "Representations/Infrastructure/Image.h"
#include "Platform/Thread.h"

class RobotConsole;
class ImageWidget;

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

  ImageWidget* widget; /**< The widget of this view */

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

  void setUndoRedo(const bool enableUndo, const bool enableRedo);
  void setDrawnColor(ColorClasses::Color color) {drawnColor = color;}
  void setDrawAllColors(bool drawAll) {drawAllColors = drawAll;}
  ColorClasses::Color getDrawnColor() const {return drawnColor;}

private:
  ImageView& imageView;
  QImage* imageData;
  int imageWidth;
  int imageHeight;
  unsigned int lastImageTimeStamp;
  unsigned int lastColorTableTimeStamp;
  unsigned int lastDrawingsTimeStamp;
  QPainter painter;
  QPoint dragStart;
  QPoint dragStartOffset;
  QPoint dragPos;
  float zoom;
  float scale = 1.f;
  QPoint offset;
  bool headControlMode;

  // which classified should be drawn?
  ColorClasses::Color drawnColor;
  bool drawAllColors;

  QAction* undoAction;
  QAction* redoAction;

  void paintEvent(QPaintEvent* event);
  virtual void paint(QPainter& painter);
  void paintDrawings(QPainter& painter);
  void copyImage(const Image& srcImage);
  void copyImageSegmented(const Image& srcImage);
  void paintImage(QPainter& painter, const Image& srcImage);
  bool needsRepaint() const;
  void window2viewport(QPoint& point);
  void mouseMoveEvent(QMouseEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void keyPressEvent(QKeyEvent* event);
  bool event(QEvent* event);
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
  void saveImg();
  void allColorsAct();
  void colorAct(int color);

private:
  /* The toolbar of a widget (and therefore the containing actions) is deleted by the
   * SimRobot mainwindow if another view receives focus. If this happens there is no
   * way to know for this widget that the toolbar (and therefor the undo/redo buttons)
   * is deleted. So this work around sets the undo/redo button pointers to nullptr if
   * they are deleted.
   */
  class WorkAroundAction : public QAction
  {
  private:
    QAction** toBeSetToNULL;
  public:
    WorkAroundAction(QAction** toBeSetToNULL, const QIcon& icon, const QString& text, QObject* parent)
      : QAction(icon, text, parent), toBeSetToNULL(toBeSetToNULL) {}
    ~WorkAroundAction()
    {
      (*toBeSetToNULL) = nullptr;
    }
  };
};
