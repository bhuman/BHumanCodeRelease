/**
 * This file implements a module that calculates the regions around interesting spots that must
 * be searched by CNS-based image processing modules.
 * @author Thomas Röfer
 */

#include "CNSRegionsProvider.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include "Math/BHMath.h"

MAKE_MODULE(CNSRegionsProvider, perception);

void CNSRegionsProvider::update(BallRegions& ballRegions)
{
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  ballRegions.regions.clear();

  if(theCameraInfo.camera == CameraInfo::lower || (theCameraInfo.camera == CameraInfo::upper && checkProcessingOfUpperImage()))
  {
    int mode = theCameraInfo.camera == CameraInfo::lower ? lowerMode : upperMode;
    if(mode == 1)
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
}

void CNSRegionsProvider::update(CNSRegions& cnsRegions)
{
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  cnsRegions.regions.clear();

  if(theCameraInfo.camera == CameraInfo::lower || (theCameraInfo.camera == CameraInfo::upper && checkProcessingOfUpperImage()))
  {
    int mode = theCameraInfo.camera == CameraInfo::lower ? lowerMode : upperMode;
    if(mode == 0)
    {
      const int xGridSize = theCameraInfo.width / blockSizeX;
      const int yGridSize = theCameraInfo.height / blockSizeY;
      grid.resize((xGridSize + 2) * (yGridSize + 2));
      stack.reserve((xGridSize + 2) * (yGridSize + 2));
      std::fill(grid.begin(), grid.end(), 0);
      auto searchGrid = [this, xGridSize](int y, int x) -> char& {return grid[y * (xGridSize + 2) + x];};

      // Add penalty mark regions
      for(const Boundaryi& region : theCNSPenaltyMarkRegions.regions)
        for(int y = region.y.min / blockSizeY; y < region.y.max / blockSizeY; ++y)
          for(int x = region.x.min / blockSizeX; x < region.x.max / blockSizeX; ++x)
            searchGrid(y + 1, x + 1) = 1;

      for(int y = 0; y < yGridSize; ++y)
        for(int x = 0; x < xGridSize; ++x)
          if(searchGrid(y + 1, x + 1))
          {
            Rangei xRange(x, x);
            Rangei yRange(y, y);
            floodFill(searchGrid, x + 1, y + 1, xRange, yRange);
            for(int yy = yRange.min; yy <= yRange.max; ++yy)
              for(int xx = xRange.min; xx <= xRange.max; ++xx)
                searchGrid(yy + 1, xx + 1) = 0;
            cnsRegions.regions.emplace_back(Rangei(xRange.min * blockSizeX, (xRange.max + 1) * blockSizeX),
                                            Rangei(yRange.min * blockSizeY, (yRange.max + 1) * blockSizeY));
          }
    }
    else if(mode == 1)
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

void CNSRegionsProvider::floodFill(const std::function<char& (int y, int x)>& searchGrid, int x, int y,
                                   Rangei& xRange, Rangei& yRange)
{
  stack.clear();
  searchGrid(y, x) = 0;
  stack.push_back(Vector2i(x, y));

  while(!stack.empty())
  {
    Vector2i point(stack.back());
    stack.pop_back();
    xRange.add(point.x() - 1);
    yRange.add(point.y() - 1);

    if(searchGrid(point.y() - 1, point.x()))
    {
      searchGrid(point.y() - 1, point.x()) = 0;
      stack.push_back(Vector2i(point.x(), point.y() - 1));
    }
    if(searchGrid(point.y() + 1, point.x()))
    {
      searchGrid(point.y() + 1, point.x()) = 0;
      stack.push_back(Vector2i(point.x(), point.y() + 1));
    }
    if(searchGrid(point.y(), point.x() - 1))
    {
      searchGrid(point.y(), point.x() - 1) = 0;
      stack.push_back(Vector2i(point.x() - 1, point.y()));
    }
    if(searchGrid(point.y(), point.x() + 1))
    {
      searchGrid(point.y(), point.x() + 1) = 0;
      stack.push_back(Vector2i(point.x() + 1, point.y()));
    }
  }
}
