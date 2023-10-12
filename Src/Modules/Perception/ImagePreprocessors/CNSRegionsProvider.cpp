/**
 * This file implements a module that calculates the regions around interesting spots that must
 * be searched by CNS-based image processing modules.
 * @author Thomas Röfer
 */

#include "CNSRegionsProvider.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include "Math/BHMath.h"

MAKE_MODULE(CNSRegionsProvider);

void CNSRegionsProvider::update(BallRegions& ballRegions)
{
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  ballRegions.regions.clear();

  if(theCameraInfo.camera != CameraInfo::upper || checkProcessingOfUpperImage())
  {
    const Vector3f pointInDistance(maxDistance, 0.f, 0.f);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointInDistance, theCameraMatrix, theCameraInfo, pointInImage))
    {
      const int minY = std::max(0, static_cast<int>(std::max(pointInImage.y(), theImageCoordinateSystem.origin.y()))) / blockSizeY * blockSizeY;
      if(minY < theCameraInfo.height)
      {
        const int rangeSizeY = theCameraInfo.height - minY;

        Vector2f relativePosition;
        Geometry::Circle ball;
        const float expectedRadius = Transformation::imageToRobotHorizontalPlane(Vector2f(theCameraInfo.width / 2, minY + rangeSizeY / 2), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
                                     && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
        const int regionsX = theCameraInfo.camera == CameraInfo::upper ? 4 : 2;
        const int regionsY = std::max(1, std::min(rangeSizeY / std::max(1, static_cast<int>(2 * expectedRadius) / blockSizeY * blockSizeY), regionsX));

        for(int x = 0; x < regionsX; ++x)
          for(int y = 0; y < regionsY; ++y)
            ballRegions.regions.emplace_back(
              Rangei(x * theCameraInfo.width / blockSizeX / regionsX * blockSizeX, (x + 1) * theCameraInfo.width / blockSizeX / regionsX * blockSizeX),
              Rangei(minY + y * rangeSizeY / blockSizeY / regionsY * blockSizeY, minY + (y + 1) * rangeSizeY / blockSizeY / regionsY * blockSizeY));
      }
    }
  }
}

void CNSRegionsProvider::update(CNSRegions& cnsRegions)
{
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  cnsRegions.regions.clear();

  if(theCameraInfo.camera != CameraInfo::upper || checkProcessingOfUpperImage())
  {
    const Vector3f pointInDistance(maxDistance, 0.f, 0.f);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointInDistance, theCameraMatrix, theCameraInfo, pointInImage))
    {
      const int minY = std::max(0, static_cast<int>(std::max(pointInImage.y(), theImageCoordinateSystem.origin.y())) / blockSizeY * blockSizeY);
      if(minY < theCameraInfo.height)
        cnsRegions.regions.emplace_back(Rangei(0, theCameraInfo.width), Rangei(minY, theCameraInfo.height));
    }
  }
}

bool CNSRegionsProvider::checkProcessingOfUpperImage()
{
  if(theCameraInfo.camera == CameraInfo::upper && dontProcessUpperImageIfProjectedBallIsOutside)
  {
    if(theFrameInfo.getTimeSince(theWorldModelPrediction.timeWhenBallLastSeen) > 1000)
      return true;
    Vector2f predictedBallInImage;
    if(Transformation::robotToImage(theWorldModelPrediction.ballPosition, theCameraMatrix, theCameraInfo, predictedBallInImage))
      if(predictedBallInImage.y() > theCameraInfo.height)
        return false;
  }
  return true;
}
