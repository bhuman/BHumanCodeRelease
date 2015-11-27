/**
 * @file Controller/Views/ColorTableView.h
 * The file implements a class to visualize the color calibration.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Controller/RobotConsole.h"
#include "Platform/Thread.h"
#include "ColorTableView.h"
#include "Controller/Visualization/OpenGLMethods.h"

ColorTableView::ColorTableView(const QString& fullName, RobotConsole& c, ColorClasses::Color color, const Vector3f& b)
  : View3D(fullName, b),
    console(c),
    color(color),
    lastTimeStamp(0)
{
}

void ColorTableView::updateDisplayLists()
{
  SYNC_WITH(console);
  OpenGLMethods::paintCubeToOpenGLList(256, 256, 256,
                                       cubeId, true,
                                       127, //scale
                                       -127, -127, -127, // offsets
                                       int(background.x() * 255) ^ 0xc0,
                                       int(background.y() * 255) ^ 0xc0,
                                       int(background.z() * 255) ^ 0xc0);
  OpenGLMethods::paintColorTable(console.colorTable, color, colorsId);
  lastTimeStamp = console.colorTableTimeStamp;
}

bool ColorTableView::needsUpdate() const
{
  return console.colorTableTimeStamp != lastTimeStamp;
}
