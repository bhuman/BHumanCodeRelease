/**
* @file OffscreenRenderer.h
* Declaration of class OffscreenRenderer
* @author Colin Graf
*/

#pragma once

#include <unordered_map>

class QGLPixelBuffer;
class QGLWidget;

/**
* @class OffscreenRenderer
* A hardware accelerated off-screen rendering module that uses the Qt 4 OpenGL library
*/
class OffscreenRenderer
{
public:
  enum Method
  {
    unknown, /**< Call prepareRendering() first */
    pixelBuffer, /**< Good. */
    frameBuffer, /**< Good. */
    hiddenWindow, /**< Well..., it works at least on some systems */
  };

  /** Destructor */
  ~OffscreenRenderer();

  /**
  * Prepares the off-screen renderer to render something of a size given with \c ensureSize. This call changes the
  * rendering context to the rendering context of the off-screen renderer.
  * @param width The maximum width.
  * @param height The maximum height.
  */
  void init();

  /**
  * Selects the OpenGL context of the off-screen renderer. Call prepareRendering() before "drawing" the OpenGL image.
  * @param width The width of an image that will be rendered using this off-screen renderer
  * @param height The height of an image that will be rendered using this off-screen renderer
  * @param sampleBuffers Are sample buffers for multi-sampling required?
  * @return Whether the OpenGL context was successfully selected
  */
  bool makeCurrent(int width, int height, bool sampleBuffers = true);

  /**
  * Reads an image from current rendering context.
  * @param image The buffer where is image will be saved to.
  * @param width The image width.
  * @param height The image height.
  */
  void finishImageRendering(void* image, int width, int height);

  /**
  * Reads a depth image from current rendering context.
  * @param image The buffer where is image will be saved to.
  * @param width The image width.
  * @param height The image height.
  */
  void finishDepthRendering(void* image, int width, int height);

  /**
  * Requests the used rendering method. Only available when prepareRendering() was called at least once.
  * @return The used rendering method.
  */
  Method getRenderingMethod() const;

  /**
  * Accesses the QGLWidget used for rendering. It can be used for creating further QGLWidgets with shared display lists and textures.
  * @return The QGLWidget used for rendering
  */
  const QGLWidget* getWidget() const {return mainGlWidget;}

private:

  /**
  * @class Buffer
  * A render buffer data specialized on rendering images of a defined size.
  */
  class Buffer
  {
  public:
    QGLWidget* glWidget;
    QGLPixelBuffer* pbuffer;
    unsigned int frameBufferId;
    unsigned int renderBufferIds[2];

    /** Default constructor */
    Buffer() : glWidget(0), pbuffer(0), frameBufferId(0) {}

    /** Destructor */
    ~Buffer();
  };

  QGLWidget* mainGlWidget = nullptr;
  bool usedMainGlWidget = false;
  std::unordered_map<unsigned int, Buffer> renderBuffers;

  /**
  * Initializes the currently selected OpenGL context for off-screen rendering
  * @param hasSharedDisplayLists Whether the currently selected context has shared display lists and textures
  */
  void initContext(bool hasSharedDisplayLists);

  bool initFrameBuffer(int width, int height, Buffer& buffer);
  bool initPixelBuffer(int width, int height, bool sampleBuffers, Buffer& buffer);
  void initHiddenWindow(int width, int height, Buffer& buffer);
};
