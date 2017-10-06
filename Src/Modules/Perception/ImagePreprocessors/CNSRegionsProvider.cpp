/**
 * This file implements a module that calculates the regions around interesting spots that must
 * be searched by CNS-based image processing modules.
 * @author Thomas Röfer
 */

#include "CNSRegionsProvider.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/BHMath.h"

MAKE_MODULE(CNSRegionsProvider, perception);

void CNSRegionsProvider::update(BallRegions& ballRegions)
{
  ASSERT(blockSizeX > 0 && theCameraInfo.width / blockSizeX <= 40);
  ASSERT(blockSizeY > 0 && theCameraInfo.height / blockSizeY <= 30);
  ASSERT(blockSizeX % 16 == 0 && blockSizeY % 16 == 0);

  DECLARE_DEBUG_DRAWING("module:CNSRegionsProvider:expectedRadius", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSRegionsProvider:feetArea", "drawingOnImage");

  ballRegions.regions.clear();

  if(dontUseUpperImageButWholeLower)
  {
    if(theCameraInfo.camera == CameraInfo::lower)
      for(int x = 0; x < 2; ++x)
        for(int y = 0; y < 2; ++y)
          ballRegions.regions.emplace_back(
            Rangei(x * theCameraInfo.width / blockSizeX / 2 * blockSizeX, (x + 1) * theCameraInfo.width / blockSizeX / 2 * blockSizeX),
            Rangei(y * theCameraInfo.height / blockSizeY / 2 * blockSizeY, (y + 1) * theCameraInfo.height / blockSizeY / 2 * blockSizeY));
  }
  else
  {
    std::vector<Vector2i> spots;
    bool predicted = false;

    std::memset(searchGrid[0], 0, sizeof(searchGrid));

    // Add at least one search area if prediction is used
    if(usePrediction && theWorldModelPrediction.ballIsValid)
    {
      const Vector3f ballInRobot(theWorldModelPrediction.ballPosition.x(), theWorldModelPrediction.ballPosition.y(), theBallSpecification.radius);
      Vector2f pointInImage;
      if(Transformation::robotToImage(ballInRobot, theCameraMatrix, theCameraInfo, pointInImage)
         && pointInImage.x() >= 0 && pointInImage.x() < theCameraInfo.width &&
         pointInImage.y() >= 0 && pointInImage.y() < theCameraInfo.height)
      {
        spots.emplace_back(static_cast<int>(pointInImage.x()), static_cast<int>(pointInImage.y()));
        predicted = true;
      }
    }

    spots.insert(spots.end(), theBallSpots.ballSpots.begin(), theBallSpots.ballSpots.end());

    for(const Vector2i& spot : spots)
    {
      const float expectedRadius = IISC::getImageBallRadiusByCenter(Vector2f(static_cast<float>(spot.x()), static_cast<float>(spot.y())),
                                   theCameraInfo, theCameraMatrix, theBallSpecification);
      CIRCLE("module:CNSRegionsProvider:expectedRadius", spot.x(), spot.y(), expectedRadius, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);

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
        ballRegions.regions.emplace_back(Rangei(xRange.min * blockSizeX, xRange.max * blockSizeX),
                                         Rangei(yRange.min * blockSizeY, yRange.max * blockSizeY));
      }
      predicted = false;
    }
  }
}

void CNSRegionsProvider::update(CNSRegions& cnsRegions)
{
  cnsRegions.regions.clear();

  if(dontUseUpperImageButWholeLower)
  {
    if(theCameraInfo.camera == CameraInfo::lower)
      cnsRegions.regions.emplace_back(Rangei(0, theCameraInfo.width), Rangei(0, theCameraInfo.height));
  }
  else
  {
    if(playersFeet)
      addPlayersFeetRegions();

    // Add penalty mark regions
    for(const Boundaryi& region : theCNSPenaltyMarkRegions.regions)
      for(int y = region.y.min / blockSizeY; y < region.y.max / blockSizeY; ++y)
        for(int x = region.x.min / blockSizeX; x < region.x.max / blockSizeX; ++x)
          searchGrid[y + 1][x + 1] = 1;

    const int xGridSize = theCameraInfo.width / blockSizeX;
    const int yGridSize = theCameraInfo.height / blockSizeY;

    for(int y = 0; y < yGridSize; ++y)
      for(int x = 0; x < xGridSize; ++x)
        if(searchGrid[y + 1][x + 1])
        {
          Rangei xRange(x, x);
          Rangei yRange(y, y);
          floodFill(x + 1, y + 1, xRange, yRange);
          for(int yy = yRange.min; yy <= yRange.max; ++yy)
            for(int xx = xRange.min; xx <= xRange.max; ++xx)
              searchGrid[yy + 1][xx + 1] = 0;
          cnsRegions.regions.emplace_back(Rangei(xRange.min * blockSizeX, (xRange.max + 1) * blockSizeX),
                                          Rangei(yRange.min * blockSizeY, (yRange.max + 1) * blockSizeY));
        }
  }
}

void CNSRegionsProvider::addPlayersFeetRegions()
{
  for(auto& player : thePlayersImagePercept.players)
  {
    // Determine rectangle for search area. This code is almost dublicated from OrientationDetermination.cpp:
    int feetMidX = (player.x1FeetOnly + player.x2FeetOnly) / 2;
    int y = player.y2;

    Vector2f pointOnField;
    Vector2f pointInImage;
    if(Transformation::imageToRobot(Vector2f(feetMidX, y), theCameraMatrix, theCameraInfo, pointOnField)
       && Transformation::robotToImage(Vector3f(pointOnField[0], pointOnField[1], searchFootHeightByHeight), theCameraMatrix, theCameraInfo, pointInImage))
    {
      int footHeight = static_cast<int>(pointInImage.y() - y
                                        + IISC::getImageDiameterByLowestPointAndFieldDiameter(searchFootHeightByDepth, Vector2f(feetMidX, y),
                                            theCameraInfo, theCameraMatrix));
      int maxRobotWidthClosedArms = static_cast<int>(IISC::getHorizontalImageDiameterByMiddlePointAndFieldDiameter(robotExpectedWidth, Vector2f(feetMidX, y), theCameraInfo, theCameraMatrix));

      // define rectangle for search:
      int feetBoxX1 = clip(feetMidX - maxRobotWidthClosedArms / 2, 0, theCameraInfo.width - 1);
      int feetBoxY1 = clip(y + 20, 0, theCameraInfo.height - 1);
      int feetBoxX2 = clip(feetMidX + maxRobotWidthClosedArms / 2, 0, theCameraInfo.width - 1);
      int feetBoxY2 = clip(y - footHeight, 0, theCameraInfo.height - 1);

      // clip rectangle:
      int x1 = std::max(feetBoxX1, 0);
      int y1 = std::max(feetBoxY2, 0);
      int x2 = std::min(feetBoxX2, theCameraInfo.width - 1);
      int y2 = std::min(feetBoxY1, theCameraInfo.height - 1);

      RECTANGLE("module:CNSRegionsProvider:feetArea", x1, y1, x2, y2, 2, Drawings::solidPen, ColorRGBA::red);

      const Rangei xRange(x1 / blockSizeX, x2 / blockSizeX + 1);
      const Rangei yRange(y1 / blockSizeY, y2 / blockSizeY + 1);

      for(int y = yRange.min; y < yRange.max; ++y)
        for(int x = xRange.min; x < xRange.max; ++x)
          searchGrid[y + 1][x + 1] = 1;
    }
  }
}

void CNSRegionsProvider::floodFill(int x, int y, Rangei& xRange, Rangei& yRange)
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
