/**
 * @file PlayersDeeptectorFeatBOPLower.cpp
 *
 * This file implements a module that transplants the SegmentedObstacleImage onto the lower camera handling of the PlayersDeeptector.
 *
 * @author Bernd Poppinga
 * @author Andreas Baude
 * @author Arne Hasselbring
 */

#include "PlayersDeeptectorFeatBOPLower.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Stopwatch.h"
#include "Math/Eigen.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(PlayersDeeptectorFeatBOPLower);

PlayersDeeptectorFeatBOPLower::PlayersDeeptectorFeatBOPLower()
{
  ASSERT(theCameraInfo.camera == CameraInfo::lower);
}

void PlayersDeeptectorFeatBOPLower::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
  theObstaclesImagePercept.obstacles = obstaclesLower;
}

void PlayersDeeptectorFeatBOPLower::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptectorFeatBOPLower:image", "drawingOnImage");

  std::vector<ObstaclesImagePercept::Obstacle>& obstacles = obstaclesLower;
  obstacles.clear();
  theObstaclesFieldPercept.obstacles.clear();
  const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.clear();

  if(!theCameraMatrix.isValid || !theFieldBoundary.isValid || theECImage.grayscaled.width * theECImage.grayscaled.height == 0 || theSegmentedObstacleImage.obstacle.width * theSegmentedObstacleImage.obstacle.height == 0)
    return;
  ASSERT(xyStep < theECImage.grayscaled.width && xyStep > 0 && xyStep < theECImage.grayscaled.height && xyStep > 0);

  std::vector<std::vector<Region>> regions(theSegmentedObstacleImage.obstacle.height, std::vector<Region>(theSegmentedObstacleImage.obstacle.width, Region()));
  STOPWATCH("module:PlayersDeeptectorFeatBOPLower:scanImage") scanImage(regions);
  STOPWATCH("module:PlayersDeeptectorFeatBOPLower:clusterRegions") dbScan(regions, obstacles);

  bool mergeObstacles = false;
  const Vector2f& robotRotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? pRobotRotationDeviationInStand : pRobotRotationDeviation;
  do
  {
    mergeObstacles = mergeLowerObstacles && !mergeObstacles && obstacles.size() >= 2;
    auto it = obstacles.begin();
    while(it != obstacles.end())
    {
      bool validObstacle;
      ObstaclesImagePercept::Obstacle& obstacleInImage = *it;
      ObstaclesFieldPercept::Obstacle obstacleOnField;

      Matrix2f leftCovariance, rightCovariance;

      if((validObstacle = (
          theMeasurementCovariance.transformWithCovLegacy(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), 0.f,
                                                          robotRotationDeviation, obstacleOnField.left, leftCovariance) &&
          theMeasurementCovariance.transformWithCovLegacy(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), 0.f,
                                                          robotRotationDeviation, obstacleOnField.right, rightCovariance))))
      {
        obstacleOnField.fallen = obstacleInImage.fallen;
        obstacleOnField.type = ObstaclesFieldPercept::unknown;
        obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

        const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
        if((validObstacle = obstacleInImage.top < static_cast<int>(theECImage.grayscaled.height / theSegmentedObstacleImage.obstacle.height) ||
                            static_cast<float>(obstacleInImage.bottom - obstacleInImage.top) / theECImage.grayscaled.height > 0.5f))
        {
          if(!mergeObstacles)
            STOPWATCH("module:PlayersDeeptectorFeatBOPLower:detectJersey")
              theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
          float minWidth = minWidthOnFieldNoMatch;
          bool hasMatch = false;
          for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
          {
            const Pose2f inverseOdometryOffset = theOdometryData.inverse() * theOtherOdometryData;
            const Rangea rangeUpper((inverseOdometryOffset * incompleteObstacle.right).angle(), (inverseOdometryOffset * incompleteObstacle.left).angle());
            if((hasMatch = (range.min <= rangeUpper.max && rangeUpper.min <= range.max)))
            {
              obstacleOnField.type = incompleteObstacle.type;
              minWidth = minWidthOnFieldNoMatch / 2;
              break;
            }
          }
          obstacleOnField.fallen = !hasMatch && (obstacleInImage.right - obstacleInImage.left >= static_cast<int>(theECImage.grayscaled.width / 2) ||
                                                 obstacleInImage.bottom - obstacleInImage.top >= static_cast<int>(theECImage.grayscaled.height / 2));

          RECTANGLE("module:PlayersDeeptectorFeatBOPLower:image", obstacleInImage.left, obstacleInImage.top, obstacleInImage.right, obstacleInImage.bottom, 3, Drawings::solidPen, ColorRGBA::violet);
          if((validObstacle = (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidth)) && !mergeObstacles)
          {
            const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
            obstacleOnField.covariance = (leftCovariance + rightCovariance) / 2;
            theObstaclesFieldPercept.obstacles.emplace_back(obstacleOnField);
          }
          else if(!validObstacle && (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidthOnFieldNoMatch / 2))
            const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
        }
      }
      it = validObstacle ? it + 1 : obstacles.erase(it);
    }

    if(mergeObstacles)
    {
      bool mergedSomething;
      do
      {
        std::vector<bool> merged(obstacles.size(), false);
        std::vector<ObstaclesImagePercept::Obstacle> mergedObstacles;
        mergedSomething = false;
        for(size_t i = 0; i < obstacles.size(); ++i)
        {
          if(merged[i]) continue;
          ObstaclesImagePercept::Obstacle& a = obstacles[i];
          const Rangei obstacleRange = Rangei(a.left, a.right);

          for(size_t j = i + 1; j < obstacles.size(); ++j)
          {
            ObstaclesImagePercept::Obstacle& b = obstacles[j];
            const Rangei otherObstacleRange = Rangei(b.left, b.right);
            if(std::min(obstacleRange.max, otherObstacleRange.max) - std::max(obstacleRange.min, otherObstacleRange.min) > 0)
            {
              a.top = std::min(a.top, b.top);
              a.bottom = std::max(a.bottom, b.bottom);
              a.left = std::min(a.left, b.left);
              a.right = std::max(a.right, b.right);
              mergedObstacles.emplace_back(a);
              merged[i] = true;
              merged[j] = true;
              mergedSomething = true;
            }
          }
          if(!merged[i]) mergedObstacles.emplace_back(a);
        }
        obstacles = mergedObstacles;
      }
      while(mergedSomething);
    }
  }
  while(mergeObstacles);
}

void PlayersDeeptectorFeatBOPLower::update(ObstaclesPerceptorData& theObstaclesPerceptorData)
{
  theObstaclesPerceptorData.cameraInfo = theCameraInfo;
  theObstaclesPerceptorData.cameraMatrix = theCameraMatrix;
  theObstaclesPerceptorData.imageCoordinateSystem = theImageCoordinateSystem;
}

bool PlayersDeeptectorFeatBOPLower::trimObstacle(bool trimHeight, ObstaclesImagePercept::Obstacle& obstacleInImage)
{
  const int minX = std::max(obstacleInImage.left, 0);
  const int minY = std::max(obstacleInImage.top, 0);
  const int maxX = std::min(obstacleInImage.right, static_cast<int>(theECImage.grayscaled.width)), maxY = std::min(obstacleInImage.bottom, static_cast<int>(theECImage.grayscaled.height));

  int stepSize = xyStep;
  std::vector<std::vector<int>> limits = {{}, {}};
  for(int y = minY; y < maxY; y += stepSize)
  {
    int offset = minX, step = xyStep;
    for(int side = 0; side < 2; ++side, offset = maxX - 1, step *= -1)
    {
      if((side == 0 && minX <= 10) || (side == 1 && maxX >= static_cast<int>(theECImage.grayscaled.width - 10)))
        continue;

      const PixelTypes::GrayscaledPixel* secondSpot = theECImage.grayscaled[y] + offset;
      short firstSpot = *secondSpot;;
      std::function<bool(int)> comp = (side == 0) ? static_cast<std::function<bool(int)>>([maxX, stepSize](int a) -> bool {return a < maxX - stepSize;})
                                                  : static_cast<std::function<bool(int)>>([minX, stepSize](int a) -> bool {return a >= minX + stepSize;});
      for(int x = offset; comp(x); x += step, firstSpot = *secondSpot)
      {
        secondSpot += step;
        if(std::abs(*secondSpot - firstSpot) > minContrastDiff / 2)
        {
          limits[side].emplace_back(x - step / 2);
          break;
        }
      }
    }
  }
  std::sort(limits[0].begin(), limits[0].end());
  if(!limits[0].empty()) obstacleInImage.left = limits[0][limits[0].size() / 2];
  std::sort(limits[1].begin(), limits[1].end());
  if(!limits[1].empty()) obstacleInImage.right = limits[1][limits[1].size() / 2];

  if(trimHeight)
  {
    const Rangei yRange = Rangei(minY + (maxY - minY) / 4, maxY - 1);
    int step = stepSize, upperY = -1, lowerY = -1, botCan = obstacleInImage.bottom;
    for(int side = 0; side < 2; ++side, step *= -1)
    {
      bool satRow = false, nonSatRow = false;
      for(int y = (minY + maxY) / 2; yRange.isInside(y) && (!nonSatRow || !satRow); y += step)
      {
        const PixelTypes::GrayscaledPixel* spot = theECImage.saturated[y] + obstacleInImage.left;
        int x = obstacleInImage.left;
        for(; x < obstacleInImage.right && *spot > satThreshold; x += stepSize, spot += stepSize);

        if(x >= obstacleInImage.right)
          satRow = true;
        else
        {
          botCan = y;
          nonSatRow = true;
        }
        if(satRow && nonSatRow)(side == 0 ? lowerY : upperY) = botCan;
      }
    }
    obstacleInImage.bottom = upperY != -1 ? upperY : (lowerY != -1 ? lowerY : obstacleInImage.bottom);
  }
  Vector2f relativePosition;
  Geometry::Circle ball;
  const float radius = Transformation::imageToRobotHorizontalPlane(Vector2f((obstacleInImage.right + obstacleInImage.left) / 2, (obstacleInImage.top + obstacleInImage.bottom) / 2), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
                       && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
  const int minWidthInImage = std::max(minPixel, static_cast<int>(radius));
  return obstacleInImage.right - obstacleInImage.left >= minWidthInImage && obstacleInImage.bottom - obstacleInImage.top >= minWidthInImage &&
         static_cast<float>(obstacleInImage.right - obstacleInImage.left) / static_cast<float>(maxX - minX) >= minBeforeAfterTrimRatio;
}

void PlayersDeeptectorFeatBOPLower::scanImage(std::vector<std::vector<Region>>& regions)
{
  const unsigned int xScale = theCameraInfo.width / theSegmentedObstacleImage.obstacle.width;
  const unsigned int yScale = theCameraInfo.height / theSegmentedObstacleImage.obstacle.height;

  std::vector<std::pair<int, int>> yLimits(theSegmentedObstacleImage.obstacle.width);
  for(unsigned int x = xScale / 2, index = 0; index < theSegmentedObstacleImage.obstacle.width; x += xScale, ++index)
    yLimits[index] = std::make_pair(theFieldBoundary.getBoundaryY(x), theBodyContour.getBottom(x, theCameraInfo.height));

  for(unsigned int y = 0; y < theSegmentedObstacleImage.obstacle.height; ++y)
  {
    const int yFullResolution = y * yScale;
    for(unsigned int x = 0; x < theSegmentedObstacleImage.obstacle.width; ++x)
    {
      const auto& [yMin, yMax] = yLimits[x];
      regions[y][x].regionIndices = Vector2i(x, y);
      if(yFullResolution > yMin && yFullResolution < yMax && theSegmentedObstacleImage.obstacle[y][x] > 128)
      {
        regions[y][x].isObstacle = true;
        regions[y][x].maxY = yFullResolution;
      }
    }
  }
}

void PlayersDeeptectorFeatBOPLower::dbScan(std::vector<std::vector<Region>>& regions, std::vector<ObstaclesImagePercept::Obstacle>& obstacles)
{
  for(std::vector<Region>& horizontalRegionLine : regions)
    for(Region& region : horizontalRegionLine)
    {
      if(!region.isObstacle || region.clustered)
        continue;
      std::vector<Vector2i> neighbors;
      regionQuery(regions, region.regionIndices, 1, neighbors);
      if(neighbors.size() >= minNeighborPoints)
      {
        std::vector<Vector2i> cluster;
        Vector2i topLeft = region.regionIndices, bottomRight = Vector2i(region.regionIndices.x(), region.maxY);
        if(expandCluster(regions, region, neighbors, cluster, topLeft, bottomRight))
        {
          ObstaclesImagePercept::Obstacle obstacle;
          obstacle.top = topLeft.y() * (theECImage.grayscaled.height / theSegmentedObstacleImage.obstacle.height);
          obstacle.bottom = bottomRight.y();
          obstacle.left = topLeft.x() * (theECImage.grayscaled.width / theSegmentedObstacleImage.obstacle.width);
          obstacle.right = (bottomRight.x() + 1) * (theECImage.grayscaled.width / theSegmentedObstacleImage.obstacle.width);
          obstacle.bottomFound = true;
          obstacle.fallen = false;

          if(!trimObstacles || trimObstacle(true, obstacle))
            obstacles.emplace_back(obstacle);
        }
      }
    }
}

bool PlayersDeeptectorFeatBOPLower::expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight)
{
  cluster.emplace_back(region.regionIndices);
  region.clustered = true;

  for(unsigned int i = 0; i < neighbors.size(); ++i)
  {
    Vector2i& neighborIndices = neighbors[i];
    Region& currentRegion = regions[neighborIndices.y()][neighborIndices.x()];
    if(currentRegion.clustered)
      continue;

    cluster.emplace_back(neighborIndices);
    currentRegion.clustered = true;
    if(currentRegion.isObstacle)
    {
      topLeft = Vector2i(std::min(currentRegion.regionIndices.x(), topLeft.x()), std::min(currentRegion.regionIndices.y(), topLeft.y()));
      bottomRight = Vector2i(std::max(currentRegion.regionIndices.x(), bottomRight.x()), std::max(currentRegion.maxY, bottomRight.y()));
    }

    std::vector<Vector2i> nextNeighbors;
    regionQuery(regions, neighborIndices, 1, nextNeighbors);
    if(nextNeighbors.size() >= minNeighborPoints)
      for(Vector2i& nextNeighbor : nextNeighbors)
        if(!regions[nextNeighbor.y()][nextNeighbor.x()].clustered)
          neighbors.emplace_back(nextNeighbor);
  }
  for(int yRegionIndex = topLeft.y() - 1; yRegionIndex >= 0 && yRegionIndex == topLeft.y() - 1; --yRegionIndex)
    for(int xRegionIndex = topLeft.x(); xRegionIndex <= bottomRight.x(); ++xRegionIndex)
      if(regions[yRegionIndex][xRegionIndex].isObstacle)
      {
        cluster.emplace_back(Vector2i(xRegionIndex, yRegionIndex));
        topLeft = Vector2i(topLeft.x(), yRegionIndex);
      }
  return true;
}

void PlayersDeeptectorFeatBOPLower::regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& regionIndices, int dis, std::vector<Vector2i>& neighbors)
{
  for(int y = std::max(regionIndices.y() - dis, 0); y < std::min(regionIndices.y() + dis + 1, static_cast<int>(regions.size())); ++y)
    for(int x = std::max(regionIndices.x() - dis, 0); x < std::min(regionIndices.x() + dis + 1, static_cast<int>(regions[y].size())); ++x)
      if(regions[y][x].isObstacle)
        neighbors.emplace_back(Vector2i(x, y));
}
