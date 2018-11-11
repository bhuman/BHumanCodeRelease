/**
 * @file Controller/Visualization/OpenGLMethods.h
 * Declaration of class OpenGLMethods.
 *
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 */

#pragma once

#include "Tools/Debugging/DebugImages.h"

class RobotConsole;

/**
 * @namespace OpenGLMethods
 *
 * Defines methods to initialize and paint with openGL
 */
namespace OpenGLMethods
{
  /** writes a cube to an openGL list*/
  void paintCubeToOpenGLList(int xLow, int yLow, int zLow, int xHigh, int yHigh, int zHigh,
                             int listID, bool paintFullCube, int scale, int offsetX, int offsetY, int offsetZ,
                             int r, int g, int b, bool beginList = true, bool endList = true);

  /** writes a cube to an openGL list*/
  inline void paintCubeToOpenGLList(int xSize, int ySize, int zSize, int listID, bool paintFullCube,
                                    int scale, int offsetX, int offsetY, int offsetZ, int r, int g, int b)
  {
    paintCubeToOpenGLList(0, 0, 0, xSize, ySize, zSize, listID, paintFullCube, scale, offsetX, offsetY, offsetZ, r, g, b);
  }

  /** writes the pixels of the image between y1 and y2 to an openGL list*/
  void paintImagePixelsToOpenGLList(RobotConsole& console, const DebugImage& image, int colorModel, int zComponent, bool polygon, int listID, int x1, int x2, int y1, int y2);

  /** writes the pixels of the image to an openGL list*/
  inline void paintImagePixelsToOpenGLList(RobotConsole& console, const DebugImage& image, int colorModel, int zComponent, bool polygon, int listID)
  {
    paintImagePixelsToOpenGLList(console, image, colorModel, zComponent, polygon, listID, 0, image.getImageWidth(), 0, image.height);
  }
};
