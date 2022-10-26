/**
 * @file ImageCoordinateSystem.cpp
 * Implementation of a struct that provides transformations on image coordinates.
 * @author Thomas Röfer
 * @author <a href="mailto:oberlies@sim.tu-darmstadt.de">Tobias Oberlies</a>
 */

#include "ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugImages.h"
#include "ImageProcessing/PixelTypes.h"
#include "ImageProcessing/Image.h"
#include "Framework/Blackboard.h"

Vector2f ImageCoordinateSystem::fromCorrected(const Vector2f& correctedCoords, const Vector2f& offset) const
{
  float y = correctedCoords.y();

  // Correction of points too far outside the image cannot be inverted correctly,
  // because they cannot be the result of toCorrected.
  if(y < -cameraInfo.height / 4 || y >= cameraInfo.height * 5 / 4)
    return correctedCoords;

  float factor;
  for(int i = 0; i < 3; ++i)
  {
    factor = a + y * b;
    float lastY = y;
    y = cameraInfo.opticalCenter.y() + std::tan(std::atan((correctedCoords.y() - cameraInfo.opticalCenter.y()) / cameraInfo.focalLengthHeight) + factor * offset.y()) * cameraInfo.focalLengthHeight;
    if(std::abs(y - lastY) < 0.5f)
      break;
  }
  return Vector2f(cameraInfo.opticalCenter.x() - std::tan(std::atan((cameraInfo.opticalCenter.x() - correctedCoords.x()) / cameraInfo.focalLength) + factor * offset.x()) * cameraInfo.focalLength, y);
}

void ImageCoordinateSystem::draw() const
{
  DEBUG_DRAWING("horizon", "drawingOnImage") // displays the horizon
  {
    ARROW("horizon", origin.x(), origin.y(),
          origin.x() + rotation(0, 0) * 100,
          origin.y() + rotation(1, 0) * 100,
          5, Drawings::solidPen, ColorRGBA::red);
    ARROW("horizon", origin.x(), origin.y(),
          origin.x() + rotation(0, 1) * 100,
          origin.y() + rotation(1, 1) * 100,
          5, Drawings::solidPen, ColorRGBA::red);
  }

  COMPLEX_IMAGE("corrected")
  {
    if(Blackboard::getInstance().exists("CameraImage"))
    {
      Image<PixelTypes::YUYVPixel> correctedImage;
      const CameraImage& theCameraImage = static_cast<const CameraImage&>(Blackboard::getInstance()["CameraImage"]);

      correctedImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
      memset(correctedImage[0], 0, (cameraInfo.width / 2) * cameraInfo.height * sizeof(PixelTypes::YUYVPixel));
      int yDest = static_cast<int>(toCorrected(Vector2i(0, 0)).y());
      for(int ySrc = 0; ySrc < cameraInfo.height; ++ySrc)
        for(int yDest2 = static_cast<int>(toCorrected(Vector2i(0, ySrc)).y()); yDest <= yDest2; ++yDest)
          if(yDest >= 0 && yDest < static_cast<int>(cameraInfo.height))
          {
            int xDest = static_cast<int>(toCorrected(Vector2i(0, ySrc)).x()) / 2;
            for(int xSrc = 0; xSrc < cameraInfo.width; xSrc += 2)
              for(int xDest2 = static_cast<int>(toCorrected(Vector2i(xSrc, ySrc)).x()) / 2; xDest <= xDest2; ++xDest)
                if(xDest >= 0 && xDest < static_cast<int>(cameraInfo.width / 2))
                  correctedImage[yDest][xDest].color = theCameraImage[ySrc][xSrc / 2].color;
          }
      SEND_DEBUG_IMAGE("corrected", correctedImage);
    }
  }
}
