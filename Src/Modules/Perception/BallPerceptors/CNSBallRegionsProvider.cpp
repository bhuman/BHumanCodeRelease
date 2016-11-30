/**
 * This file implements a module that calculates the regions around ball spots that must be scanned
 * for a ball detection.
 * @author Thomas Röfer
 */

#include "CNSBallRegionsProvider.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(CNSBallRegionsProvider, perception);

void CNSBallRegionsProvider::update(BallRegions& ballRegions)
{
  ASSERT(blockSizeX > 0 && theCameraInfo.width / blockSizeX <= 40);
  ASSERT(blockSizeY > 0 && theCameraInfo.height / blockSizeY <= 30);
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  DECLARE_DEBUG_DRAWING("module:CNSImageProvider:expectedRadius", "drawingOnImage");

  ballRegions.regions.clear();
  std::vector<Vector2i> spots;
  bool predicted = false;

  std::memset(searchGrid[0], 0, sizeof(searchGrid));

  // Add at least one search area if prediction is used
  if(usePrediction && theBallPrediction.isValid)
  {
    const Vector3f ballInRobot(theBallPrediction.position.x(), theBallPrediction.position.y(), theFieldDimensions.ballRadius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(ballInRobot, theCameraMatrix, theCameraInfo, pointInImage)
       && pointInImage.x() >= 0 && pointInImage.x() < theCameraInfo.width &&
       pointInImage.y() >= 0 && pointInImage.y() < theCameraInfo.height)
    {
      spots.emplace_back(Vector2i(static_cast<int>(pointInImage.x()), static_cast<int>(pointInImage.y())));
      predicted = true;
    }
  }

  spots.insert(spots.end(), theBallSpots.ballSpots.begin(), theBallSpots.ballSpots.end());

  for(const Vector2i& spot : spots)
  {
    const float expectedRadius = IISC::getImageBallRadiusByCenter(Vector2f(static_cast<float>(spot.x()), static_cast<float>(spot.y())),
                                                                  theCameraInfo, theCameraMatrix, theFieldDimensions);
    CIRCLE("module:CNSBallRegionsProvider:expectedRadius", spot.x(), spot.y(), expectedRadius, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);

    {
      const int radius = static_cast<int>(std::ceil(expectedRadius * (predicted ? predictedFactor : sizeFactor)));
      const Vector2i upperLeft(std::max(spot.x() - radius, 0), std::max(spot.y() - radius, 0));
      const Vector2i lowerRight(std::min(spot.x() + radius, theCameraInfo.width - 1), std::min(spot.y() + radius, theCameraInfo.height - 1));
      const Rangei xRange(upperLeft.x() / blockSizeX, lowerRight.x() / blockSizeX + 1);
      const Rangei yRange(upperLeft.y() / blockSizeY, lowerRight.y() / blockSizeY + 1);

      for(int y = yRange.min; y < yRange.max; ++y)
        for(int x = xRange.min; x < xRange.max; ++x)
          searchGrid[y + 1][x + 1] = 1;
    }

    if(!predicted)
    {
      const int radius = static_cast<int>(std::ceil(expectedRadius * (sizeFactor - 1.f)));
      const Vector2i upperLeft(std::max(spot.x() - radius, 0), std::max(spot.y() - radius, 0));
      const Vector2i lowerRight(std::min(spot.x() + radius, theCameraInfo.width - 1), std::min(spot.y() + radius, theCameraInfo.height - 1));
      const Rangei xRange(upperLeft.x() / blockSizeX, lowerRight.x() / blockSizeX + 1);
      const Rangei yRange(upperLeft.y() / blockSizeY, lowerRight.y() / blockSizeY + 1);
      ballRegions.regions.emplace_back(Boundaryi(Rangei(xRange.min * blockSizeX, xRange.max * blockSizeX),
                                                 Rangei(yRange.min * blockSizeY, yRange.max * blockSizeY)));
    }
    predicted = false;
  }
}

void CNSBallRegionsProvider::update(CNSRegions& cnsRegions)
{
  cnsRegions.regions.clear();

  const int xGridSize = theCameraInfo.width / blockSizeX;
  const int yGridSize = theCameraInfo.height / blockSizeY;

  for(int y = 0; y < yGridSize; ++y)
    for(int x = 0; x < xGridSize; ++x)
      if(searchGrid[y + 1][x + 1])
      {
        Rangei xRange(x, x);
        Rangei yRange(y, y);
        floodFill(x + 1, y + 1, xRange, yRange);
        cnsRegions.regions.emplace_back(Boundaryi(Rangei(xRange.min * blockSizeX, (xRange.max + 1) * blockSizeX),
                                                  Rangei(yRange.min * blockSizeY, (yRange.max + 1) * blockSizeY)));
      }
}

void CNSBallRegionsProvider::floodFill(int x, int y, Rangei& xRange, Rangei& yRange)
{
  stack.clear();
  searchGrid[y][x] = 0;
  stack.push_back(Vector2i(x, y));

  while(!stack.empty())
  {
    Vector2i point(stack.back());
    stack.pop_back();
    xRange.add(point.x() - 1);
    yRange.add(point.y() - 1);

    if(searchGrid[point.y() - 1][point.x()])
    {
      searchGrid[point.y() - 1][point.x()] = 0;
      stack.push_back(Vector2i(point.x(), point.y() - 1));
    }
    if(searchGrid[point.y() + 1][point.x()])
    {
      searchGrid[point.y() + 1][point.x()] = 0;
      stack.push_back(Vector2i(point.x(), point.y() + 1));
    }
    if(searchGrid[point.y()][point.x() - 1])
    {
      searchGrid[point.y()][point.x() - 1] = 0;
      stack.push_back(Vector2i(point.x() - 1, point.y()));
    }
    if(searchGrid[point.y()][point.x() + 1])
    {
      searchGrid[point.y()][point.x() + 1] = 0;
      stack.push_back(Vector2i(point.x() + 1, point.y()));
    }
  }
}
