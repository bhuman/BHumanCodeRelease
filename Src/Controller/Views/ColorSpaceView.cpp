/**
 * @file Controller/Views/ColorSpaceView.cpp
 *
 * Implementation of class ColorSpaceView
 *
 * @author Thomas Röfer
 */

#include "ColorSpaceView.h"
#include "Controller/RobotConsole.h"
#include "Controller/Visualization/OpenGLMethods.h"
#ifdef MACOS
#include <gl.h>
#else
#include <GL/gl.h>
#endif

ColorSpaceView::ColorSpaceView(const QString& fullName, RobotConsole& c, const std::string& n, ColorModel cm, int ch, const Vector3f& b, const std::string& threadIdentifier) :
  View3D(fullName, b), console(c), name(n), colorModel(cm), channel(ch), threadIdentifier(threadIdentifier)
{}

void ColorSpaceView::updateDisplayLists()
{
  SYNC_WITH(console);
  DebugImage* image = nullptr;
  DebugImage* raw = nullptr;

  RobotConsole::Images& currentImages = console.threadData[threadIdentifier].images;
  RobotConsole::Images::const_iterator i = currentImages.find(name);

  if(i != currentImages.end())
    image = i->second.image;
  i = currentImages.find("raw image");
  if(i != currentImages.end())
    raw = i->second.image;
  if(image && (channel < 3 || raw))
  {
    if(!channel)
      OpenGLMethods::paintCubeToOpenGLList(256, 256, 256,
                                           cubeId, true,
                                           127, //scale
                                           -127, -127, -127, // offsets
                                           int(background.x() * 255) ^ 0xc0,
                                           int(background.y() * 255) ^ 0xc0,
                                           int(background.z() * 255) ^ 0xc0);
    else
      OpenGLMethods::paintCubeToOpenGLList(image->getImageWidth(), image->height, 128,
                                           cubeId, true,
                                           127, //scale
                                           -image->getImageWidth() / 2, -image->height / 2, -65, // offsets
                                           int(background.x() * 255) ^ 0xc0,
                                           int(background.y() * 255) ^ 0xc0,
                                           int(background.z() * 255) ^ 0xc0);
    if(image->width > 0)
      OpenGLMethods::paintImagePixelsToOpenGLList(console, *image, colorModel, channel - 1, false, colorsId);
    lastTimestamp = image->timestamp;
  }
  else
  {
    glNewList(cubeId, GL_COMPILE_AND_EXECUTE);
    glEndList();
    glNewList(colorsId, GL_COMPILE_AND_EXECUTE);
    glEndList();
    lastTimestamp = 0;
  }
}

bool ColorSpaceView::needsUpdate() const
{
  SYNC_WITH(console);
  DebugImage* image = nullptr;
  RobotConsole::Images& currentImages = console.threadData[threadIdentifier].images;
  RobotConsole::Images::const_iterator i = currentImages.find(name);
  if(i != currentImages.end())
    image = i->second.image;
  return ((image && image->timestamp != lastTimestamp) ||
          (!image && lastTimestamp));
}
