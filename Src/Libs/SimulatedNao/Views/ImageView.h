/**
 * @file SimulatedNao/Views/ImageView.h
 *
 * This file declares a view that displays an image overlaid by debug drawing.
 * Such a view is usually associated with a thread. It displays an image provided
 * by that thread. It prefers drawings originating from its associated thread. It
 * ignores drawings from other threads if the associated thread could also provide
 * them, but currently does not. It is also possible to create a view without an
 * image. Such a view can also be independent from a specific thread.
 *
 * @author Thomas Röfer
 * @author Colin Graf
 */

#pragma once

#include <QImage>
#include <QMenu>

#include "Representations/Infrastructure/CameraImage.h"
#include "DrawingView.h"

class SACControlWidget;

/**
 * @class ImageView
 *
 * A class to represent a view displaying camera images and overlaid debug drawings.
 *
 * @author Thomas Röfer
 */
class ImageView : public DrawingView
{
  friend class SACControlWidget;

public:
  /**
   * @param fullName The path to this view in the scene graph.
   * @param console The console object.
   * @param background The name of the background image.
   * @param name The name of the view.
   * @param threadName The name of the associated thread. If empty, no thread is associated.
   * @param gain The intensity is multiplied with this factor.
   * @param ddScale The debug drawings are multiplied with this factor.
   */
  ImageView(const QString& fullName, RobotConsole& console, const std::string& background,
            const std::string& name, const std::string& threadName, float gain = 1.0f, float ddScale = 1.0f);

protected:
  /**
   * The method returns a new instance of a widget for this direct view.
   * The caller has to delete this instance. (Qt handles this)
   * @return The widget.
   */
  SimRobot::Widget* createWidget() override;

private:
  const std::string background; /**< The name of the background image. */
  float gain; /**< The intensity is multiplied with this factor. */
  float ddScale; /**< The debug drawings are multiplied with this factor. */

  friend class ImageWidget;
};

class ImageWidget : public DrawingWidget
{
  Q_OBJECT

public:
  ImageWidget(ImageView& view);
  ~ImageWidget() override;

protected:
  void paint(QPainter& painter) override;
  void copyImage(const DebugImage& srcImage);
  void paintImage(QPainter& painter, const DebugImage& srcImage, unsigned timestamp);
  bool needsRepaint() const override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  std::vector<std::pair<std::string, const DebugDrawing*>> getDrawings(const std::string& name) const override;
  QSize sizeHint() const override {return viewSize;}
  QMenu* createUserMenu() const override;

  QImage* imageData = nullptr;
  void* imageDataStorage = nullptr;
  unsigned int lastImageTimestamp = 0;

  friend class ImageView;

private slots:
  void saveImg();
};
