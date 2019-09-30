/**
 * @file OffscreenRenderer.cpp
 * An implementation of a hardware accelerated off-screen rendering module.
 * @author Colin Graf
 */

#include "Platform/OpenGL.h"
#include <QGLPixelBuffer>
#include <QGLWidget>

#include "Platform/OffscreenRenderer.h"
#include "Platform/Assert.h"
#include "Simulation/Simulation.h"
#include "Simulation/Scene.h"
#include "SimObjectWidget.h"

OffscreenRenderer::~OffscreenRenderer()
{
  if(mainGlWidget)
    mainGlWidget->makeCurrent();
  renderBuffers.clear();
  if(mainGlWidget)
    delete mainGlWidget;
}

OffscreenRenderer::Buffer::~Buffer()
{
  if(glWidget)
    delete glWidget;
  if(pbuffer)
    delete pbuffer;
}

bool OffscreenRenderer::makeCurrent(int width, int height, bool sampleBuffers)
{
  ASSERT(mainGlWidget);

  // Considering weak graphics cards glClear is faster when the color and depth buffers are not greater then they have to be.
  // So we create an individual buffer for each size in demand.

  std::unordered_map<unsigned int, Buffer>::iterator it = renderBuffers.find(width << 16 | height << 1 | (sampleBuffers ? 1 : 0));
  if(it == renderBuffers.end())
  {
    Buffer& buffer = renderBuffers[width << 16 | height << 1 | (sampleBuffers ? 1 : 0)];

    if(!initPixelBuffer(width, height, sampleBuffers, buffer))
      if(!initFrameBuffer(width, height, buffer))
        initHiddenWindow(width, height, buffer);

    return true;
  }
  else
  {
    Buffer& buffer = it->second;
    if(buffer.pbuffer)
      return buffer.pbuffer->makeCurrent();
    if(buffer.frameBufferId)
    {
      mainGlWidget->makeCurrent();
      glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, buffer.frameBufferId);
      return true;
    }
    if(buffer.glWidget)
      buffer.glWidget->makeCurrent();
    else
      mainGlWidget->makeCurrent();
    return true;
  }
}

bool OffscreenRenderer::initFrameBuffer(int width, int height, Buffer& buffer)
{
  mainGlWidget->makeCurrent();

#ifndef MACOS
  if(!GLEW_EXT_framebuffer_object)
    return false;
#endif

  glGenFramebuffersEXT(1, &buffer.frameBufferId);
  glGenRenderbuffersEXT(2, buffer.renderBufferIds);

  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, buffer.frameBufferId);

  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, buffer.renderBufferIds[0]);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, width, height);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, buffer.renderBufferIds[1]);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB, width, height);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, buffer.renderBufferIds[0]);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, buffer.renderBufferIds[1]);

  GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if(status != GL_FRAMEBUFFER_COMPLETE_EXT)
  {
    glDeleteFramebuffersEXT(1, &buffer.frameBufferId);
    glDeleteRenderbuffersEXT(2, buffer.renderBufferIds);
    buffer.frameBufferId = 0;
    return false;
  }

  return true;
}

bool OffscreenRenderer::initPixelBuffer(int width, int height, bool sampleBuffers, Buffer& buffer)
{
  const QGLFormat format(QGL::NoStencilBuffer | QGL::SingleBuffer | (sampleBuffers ? QGL::SampleBuffers : QGL::NoSampleBuffers));
  buffer.pbuffer = new QGLPixelBuffer(QSize(width, height), format, mainGlWidget);
  if(!buffer.pbuffer->isValid())
  {
    delete buffer.pbuffer;
    buffer.pbuffer = nullptr;
    return false;
  }

  VERIFY(buffer.pbuffer->makeCurrent());
  initContext(true);
  return true;
}

void OffscreenRenderer::initHiddenWindow(int width, int height, Buffer& buffer)
{
  if(!usedMainGlWidget)
  {
    mainGlWidget->setFixedSize(width, height);
    usedMainGlWidget = true;
    return;
  }

  const QGLFormat format(QGL::NoStencilBuffer | QGL::SingleBuffer);
  buffer.glWidget = new QGLWidget(format, nullptr, mainGlWidget, Qt::WindowStaysOnTopHint);
  buffer.glWidget->setFixedSize(width, height);
  buffer.glWidget->makeCurrent();
  initContext(buffer.glWidget->isSharing());
}

void OffscreenRenderer::initContext(bool hasSharedDisplayLists)
{
#ifndef MACOS
  glewInit();
#endif

  Simulation::simulation->scene->createGraphics(hasSharedDisplayLists);
}

void OffscreenRenderer::init()
{
  ASSERT(!mainGlWidget);

  const QGLFormat format(QGL::NoStencilBuffer | QGL::SingleBuffer);
  mainGlWidget = new QGLWidget(format, nullptr, nullptr, Qt::WindowStaysOnTopHint);
  mainGlWidget->makeCurrent();
  initContext(false);
}

void OffscreenRenderer::finishImageRendering(void* image, int w, int h)
{
  const int lineSize = w * 3;
  glPixelStorei(GL_PACK_ALIGNMENT, lineSize & (8 - 1) ? (lineSize & (4 - 1) ? 1 : 4) : 8);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, image);
}

void OffscreenRenderer::finishDepthRendering(void* image, int w, int h)
{
  glPixelStorei(GL_PACK_ALIGNMENT, w * 4 & (8 - 1) ? 4 : 8);
  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, image);
}

OffscreenRenderer::Method OffscreenRenderer::getRenderingMethod() const
{
  if(renderBuffers.empty())
    return unknown;
  const Buffer& buffer = renderBuffers.begin()->second;
  if(buffer.pbuffer)
    return pixelBuffer;
  if(buffer.frameBufferId)
    return frameBuffer;
  return hiddenWindow;
}
