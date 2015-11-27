/**
 * @file ImageCoordinateSystem.cpp
 * Implementation of a struct that provides transformations on image coordinates.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
 */

#include "ImageCoordinateSystem.h"
#include "Tools/Debugging/DebugDrawings.h"

ImageCoordinateSystem::~ImageCoordinateSystem()
{
  if(xTable)
  {
    delete[] xTable;
    delete[] yTable;
  }
}

void ImageCoordinateSystem::setCameraInfo(const CameraInfo& cameraInfo)
{
  this->cameraInfo = cameraInfo;
  if(xTable)
    return;
  xTable = new int[Image::maxResolutionWidth];
  yTable = new int[Image::maxResolutionHeight];
  const float focalLength = cameraInfo.focalLength * Image::maxResolutionWidth / cameraInfo.width;
  for(int i = 0; i < Image::maxResolutionWidth; ++i)
    xTable[i] = int(::atan((Image::maxResolutionWidth / 2 - i) / focalLength) * 1024 + 0.5f);
  for(int i = 0; i < Image::maxResolutionHeight; ++i)
    yTable[i] = int(::atan((i - Image::maxResolutionHeight / 2) / focalLength) * 1024 + 0.5f);
  for(int i = -3072; i < 3072; ++i)
    table[i + 3072] = int(::tan(i / 1024.0f) * focalLength + 0.5f);
}

void ImageCoordinateSystem::draw() const
{
  DECLARE_DEBUG_DRAWING("horizon", "drawingOnImage"); // displays the horizon
  ARROW("horizon", origin.x(), origin.y(),
        origin.x() + rotation(0, 0) * 50,
        origin.y() + rotation(1, 0) * 50,
        0, Drawings::solidPen, ColorRGBA::red);
  ARROW("horizon", origin.x(), origin.y(),
        origin.x() + rotation(0, 1) * 50,
        origin.y() + rotation(1, 1) * 50,
        0, Drawings::solidPen, ColorRGBA::red);
}
