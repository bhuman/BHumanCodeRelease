/**
 * @file Controller/Views/ColorReferenceView.h
 * The file implements a class to visualize the color reference.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Platform/Thread.h"
#include "ColorReferenceView.h"
#include "Controller/RobotConsole.h"
#include "Controller/Visualization/OpenGLMethods.h"

ColorReferenceView::ColorReferenceView(const QString& fullName, RobotConsole& c, ColorClasses::Color color, const Vector3<>& b)
  : View3D(fullName, b),
    console(c),
    color(color),
    lastTimeStamp(0)
{
}

void ColorReferenceView::updateDisplayLists()
{
  SYNC_WITH(console);
  OpenGLMethods::paintCubeToOpenGLList(256, 256, 256,
                                       cubeId, true,
                                       127, //scale
                                       -127, -127, -127, // offsets
                                       int(background.x * 255) ^ 0xc0,
                                       int(background.y * 255) ^ 0xc0,
                                       int(background.z * 255) ^ 0xc0);
  OpenGLMethods::paintColorReference(console.colorReference, color, colorsId);
  lastTimeStamp = console.colorReferenceTimeStamp;
}

bool ColorReferenceView::needsUpdate() const
{
  return console.colorReferenceTimeStamp != lastTimeStamp;
}
