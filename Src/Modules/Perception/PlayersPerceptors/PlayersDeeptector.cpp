/**
 * @file PlayersDeeptector.cpp
 *
 * This file implements a module that detects players in images with a neural network.
 *
 * @author Bernd Poppinga
 */

#include "PlayersDeeptector.h"
#include "Platform/File.h"
#include "Representations/Modeling/LabelImage.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"
#include "Tools/NeuralNetwork/SimpleNN.h"
#include "Tools/NeuralNetwork/Model.h"

MAKE_MODULE(PlayersDeeptector, perception)

PlayersDeeptector::PlayersDeeptector()
{
  NeuralNetwork::CompilationSettings settings;
  settings.useExpApproxInSigmoid = false;
  settings.useExpApproxInTanh = false;

  model = std::make_unique<NeuralNetwork::Model>("NeuralNets/PlayersDeeptector/players_deeptector.model");
  model->setInputUInt8(0);
  convModel.compile(*model, settings);
  ASSERT(convModel.numOfInputs() == 1);
  ASSERT(convModel.input(0).rank() == 3);
  patchSize(0) = convModel.input(0).dims(1); // width
  patchSize(1) = convModel.input(0).dims(0); // height
  ASSERT(convModel.input(0).dims(2) == 1);
  ASSERT(convModel.numOfOutputs() == 1);
  ASSERT(convModel.output(0).rank() == 3);
  ASSERT(convModel.output(0).dims(2) == 4 * 6);
  anchors.row(0) = Vector2f(0.5f, 1.f);
  anchors.row(1) = Vector2f(1.f, 2.f);
  anchors.row(2) = Vector2f(2.f, 4.f);
  anchors.row(3) = Vector2f(3.f, 6.f);
}

void PlayersDeeptector::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
  theObstaclesImagePercept.obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
}

void PlayersDeeptector::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:jersey", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:regions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:reclassifiedRegions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:clusters", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:convexHull", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:trimSpots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:trimBoxes", "drawingOnImage");

  std::vector<ObstaclesImagePercept::Obstacle>& obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
  obstacles.clear();
  theObstaclesFieldPercept.obstacles.clear();
  const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.clear();

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    LabelImage labelImage;
    if(!convModel.valid() || !theFieldBoundary.isValid || theThumbnail.imageY.height * theThumbnail.imageY.width == 0)
      return;
    memcpy(reinterpret_cast<unsigned char*>(convModel.input(0).data()), theThumbnail.imageY[0], theThumbnail.imageY.width * theThumbnail.imageY.height * sizeof(unsigned char));
    PatchUtilities::normalizeContrast<unsigned char>(reinterpret_cast<unsigned char*>(convModel.input(0).data()), patchSize, 0.02f);
    STOPWATCH("module:PlayersDeeptector:apply")
      convModel.apply();

    for(unsigned y = 0; y < convModel.output(0).dims(0) ; y++)
      for(unsigned x = 0; x < convModel.output(0).dims(1); x++)
      {
        Eigen::Map<Eigen::Matrix<float, 4, 6, Eigen::RowMajor>> pred(convModel.output(0).data() + (y * convModel.output(0).dims(1) + x) * 4 * 6);
        pred.array() = 1.f / (1.f + (pred * -1).array().exp());

        pred.col(0) = (x + pred.col(0).array()).matrix() / convModel.output(0).dims(1) * theCameraInfo.width;
        pred.col(1) = (y + pred.col(1).array()).matrix() / convModel.output(0).dims(0) * theCameraInfo.height;
        pred.col(2).array() *= 10 * anchors.col(0).array() / convModel.output(0).dims(1) * theCameraInfo.width;
        pred.col(3).array() *= 10 * anchors.col(1).array() / convModel.output(0).dims(0) * theCameraInfo.height;
        pred.col(5).array() *= 10;

        for(unsigned b = 0; b < 4; b++)
        {
          if(pred(b, 4) > objectThres)
          {
            LabelImage::Annotation box;
            box.upperLeft = Vector2f(pred(b, 0) - pred(b, 2) / 2, pred(b, 1) - pred(b, 3) / 2);
            box.lowerRight = Vector2f(pred(b, 0) + pred(b, 2) / 2, pred(b, 1) + pred(b, 3) / 2);
            box.probability = pred(b, 4);
            box.distance = pred(b, 5);
            //box.orientation = pred(b, 6);
            box.mesuredDistance = box.distance;

            Vector2f point = Vector2f((box.lowerRight.x() + box.upperLeft.x()) / 2, box.lowerRight.y());
            if(box.lowerRight.y() < theCameraInfo.height - 50 && Transformation::imageToRobot((Vector2i)point.cast<int>(), theCameraMatrix, theCameraInfo, point))
            {
              box.mesuredDistance = point.norm() / 1000;
              /*if(box.distance / box.mesuredDistance  > 2.0f || box.mesuredDistance  / box.distance  > 2.0f)
                continue;*/
            }
            if(theFieldBoundary.getBoundaryY(static_cast<int>((box.lowerRight.x() + box.upperLeft.x()) / 2.f)) > box.lowerRight.y())
              continue;
            labelImage.annotations.push_back(box);
          }
        }
      }
    labelImage.nonMaximumSuppression(0.3f);
    labelImage.bigBoxSuppression();

    for(const LabelImage::Annotation& box : labelImage.annotations)
    {
      DRAWTEXT("module:PlayersDeeptector:image", box.upperLeft.x(), box.upperLeft.y() - 2, 10, ColorRGBA::black, "dist: " << 0.001 * static_cast<float>(static_cast<int>(1000 * box.distance)) << "m");
      DRAWTEXT("module:PlayersDeeptector:image", box.upperLeft.x(), box.upperLeft.y() - 12, 10, ColorRGBA::black, "prob: " << box.probability);
      obstacles.push_back(ObstaclesImagePercept::Obstacle());
      obstacles.back().top = static_cast<int>(box.upperLeft.y());
      obstacles.back().bottom = static_cast<int>(box.lowerRight.y());
      obstacles.back().left = static_cast<int>(box.upperLeft.x());
      obstacles.back().right = static_cast<int>(box.lowerRight.x());
      obstacles.back().probability = box.probability;
      obstacles.back().distance = box.distance;
      obstacles.back().bottomFound = true;
    }
  }
  else
  {
    if(!theCameraMatrix.isValid || !theFieldBoundary.isValid || theECImage.grayscaled.width * theECImage.grayscaled.height == 0)
      return;
    ASSERT(xyStep < theECImage.grayscaled.width && xyStep > 0 && xyStep < theECImage.grayscaled.height && xyStep > 0);

    std::vector<std::vector<Region> > regions(xyRegions, std::vector<Region>(xyRegions, Region()));
    std::vector<Cluster> clusters;
    STOPWATCH("module:PlayersDeeptector:scanImage")  scanImage(regions);
    STOPWATCH("module:PlayersDeeptector:classifyRegions")  classifyRegions(regions);
    STOPWATCH("module:PlayersDeeptector:discardHomogenAreas")  discardHomogenAreas(regions);
    STOPWATCH("module:PlayersDeeptector:clusterRegions")  dbscan(regions, clusters);
    STOPWATCH("module:PlayersDeeptector:calcConvexHulls")  calcConvexHulls(clusters, obstacles, regions);
  }

  auto it = obstacles.begin();
  while(it != obstacles.end())
  {
    bool validObstacle;
    ObstaclesImagePercept::Obstacle& obstacleInImage = *it;
    ObstaclesFieldPercept::Obstacle obstacleOnField;

    if((validObstacle = (Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.left) &&
                         Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.right))))
    {
      RECTANGLE("module:PlayersDeeptector:image", obstacleInImage.left, obstacleInImage.top, obstacleInImage.right, obstacleInImage.bottom, 3, Drawings::solidPen, ColorRGBA::violet);

      obstacleOnField.fallen = obstacleInImage.fallen;
      obstacleOnField.type = ObstaclesFieldPercept::unknown;
      obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

      const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
      if(theCameraInfo.camera == CameraInfo::upper)
      {
        if(obstacleInImage.bottom > theECImage.grayscaled.height*0.9f)
          for(const ObstaclesFieldPercept::Obstacle &incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
            if((range.isInside((theOdometer.odometryOffset.inverse() * incompleteObstacle.center).angle())))
              obstacleInImage.bottomFound = false;

        if(!obstacleInImage.bottomFound)
        {
          detectJersey(obstacleInImage, obstacleOnField);
          const_cast<ObstaclesPerceptorData &>(theObstaclesPerceptorData).incompleteObstacles.push_back(obstacleOnField);
        }
        else if((validObstacle = obstacleInImage.right-obstacleInImage.left < static_cast<int>(theECImage.grayscaled.width/2)))
        {
          if(trimObstacles)  STOPWATCH("module:PlayersDeeptector:trimObstacle")  trimObstacle(obstacleInImage);
          if(!trimObstacles || (validObstacle = (obstacleInImage.right-obstacleInImage.left >= minWidthInPixel && obstacleInImage.bottom-obstacleInImage.top >= minWidthInPixel &&
                                                 Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.left) &&
                                                 Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.right) &&
                                                 (obstacleOnField.right-obstacleOnField.left).squaredNorm() <= sqr(750.f))))
          {
            obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;
            detectJersey(obstacleInImage, obstacleOnField);
            theObstaclesFieldPercept.obstacles.push_back(obstacleOnField);
          }
        }
      }
      else if((validObstacle = obstacleInImage.top < static_cast<int>(theECImage.grayscaled.height/xyRegions) || static_cast<float>(obstacleInImage.bottom-obstacleInImage.top)/theECImage.grayscaled.height > 0.5f))
      {
        detectJersey(obstacleInImage, obstacleOnField);
        float minWidth = minWidthOnFieldNoMatch;
        bool hasMatch = false;
        for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
        {
          const Rangea rangeUpper((theOdometer.odometryOffset.inverse() * incompleteObstacle.right).angle(), (theOdometer.odometryOffset.inverse() * incompleteObstacle.left).angle());
          if((hasMatch = range.isInside((theOdometer.odometryOffset.inverse() * incompleteObstacle.center).angle()) || rangeUpper.isInside(obstacleOnField.center.angle())))
          {
            obstacleOnField.type = incompleteObstacle.type;
            minWidth = minWidthOnFieldMatch;
            break;
          }
        }
        obstacleOnField.fallen = !hasMatch && (obstacleInImage.right-obstacleInImage.left >= static_cast<int>(theECImage.grayscaled.width/2) ||
                                               obstacleInImage.bottom-obstacleInImage.top >= static_cast<int>(theECImage.grayscaled.height/2));

        if((validObstacle = (obstacleOnField.right-obstacleOnField.left).squaredNorm() >= sqr(minWidth)))
        {
          const_cast<ObstaclesPerceptorData &>(theObstaclesPerceptorData).incompleteObstacles.push_back(obstacleOnField);
          COMPLEX_DRAWING("module:PlayersDeeptector:convexHull")
            for(unsigned int i = 0; i < obstacleInImage.convexHull.size(); ++i)
              LINE("module:PlayersDeeptector:convexHull", obstacleInImage.convexHull[i].x(), obstacleInImage.convexHull[i].y(), obstacleInImage.convexHull[(i+1)%obstacleInImage.convexHull.size()].x(),
                    obstacleInImage.convexHull[(i+1)%obstacleInImage.convexHull.size()].y(), 2, Drawings::solidPen, ColorRGBA::red);
          theObstaclesFieldPercept.obstacles.push_back(obstacleOnField);
        }
        else if((obstacleOnField.right-obstacleOnField.left).squaredNorm() >= sqr(minWidthOnFieldMatch))
          const_cast<ObstaclesPerceptorData &>(theObstaclesPerceptorData).incompleteObstacles.push_back(obstacleOnField);
      }
    }
    it = validObstacle ? it + 1 : obstacles.erase(it);
  }
}

void PlayersDeeptector::update(ObstaclesPerceptorData& theObstaclesPerceptorData)
{
  theObstaclesPerceptorData.cameraInfo = theCameraInfo;
  theObstaclesPerceptorData.cameraMatrix = theCameraMatrix;
  theObstaclesPerceptorData.imageCoordinateSystem = theImageCoordinateSystem;
}

void PlayersDeeptector::trimObstacle(ObstaclesImagePercept::Obstacle& obstacleInImage)
{
  const unsigned int minX = std::max(obstacleInImage.left, 0);
  const unsigned int minY = theCameraInfo.camera == CameraInfo::lower ? std::max(obstacleInImage.top, 0) : std::max(static_cast<int>(obstacleInImage.top+(obstacleInImage.bottom-obstacleInImage.top)*0.5f), 0);
  const unsigned int maxX = std::min(obstacleInImage.right, static_cast<int>(theECImage.grayscaled.width)), maxY = std::min(obstacleInImage.bottom, static_cast<int>(theECImage.grayscaled.height));
  RECTANGLE("module:PlayersDeeptector:trimBoxes", minX, minY, maxX, maxY, 3, Drawings::solidPen, ColorRGBA::violet);

  int stepSize = xyStep;
  std::vector<int> leftLimit, rightLimit;
  for(unsigned int y = minY; y < maxY; y += stepSize)
  {
    const PixelTypes::GrayscaledPixel* pixRow = theECImage.grayscaled[y];
    if(minX > 10)
    {
      short leftSpot = *(pixRow+minX);
      for(unsigned int x = minX; x < maxX-stepSize; x += stepSize)
      {
        short rightSpot = *(pixRow+x+stepSize);
        if(std::abs(rightSpot-leftSpot) > minContrastDif/3)
        {
          leftLimit.emplace_back(x-stepSize/2);
          LINE("module:PlayersDeeptector:trimSpots", x, y, x-stepSize, y, 1, Drawings::solidPen, ColorRGBA::violet);
          break;
        }
        leftSpot = rightSpot;
      }
    }

    if(maxX < theECImage.grayscaled.width-10)
    {
      short rightSpot = *(pixRow+maxX-1);
      for(unsigned int x = maxX-1; x >= minX+stepSize; x -= stepSize)
      {
        short leftSpot = *(pixRow+x-stepSize);
        if(std::abs(rightSpot-leftSpot) > minContrastDif/3)
        {
          rightLimit.emplace_back(x);
          LINE("module:PlayersDeeptector:trimSpots", x, y, x-stepSize, y, 1, Drawings::solidPen, ColorRGBA::violet);
          break;
        }
        rightSpot = leftSpot;
      }
    }
  }
  std::sort(leftLimit.begin(), leftLimit.end());
  if(leftLimit.size() > 0) obstacleInImage.left = leftLimit[leftLimit.size()/2];
  std::sort(rightLimit.begin(), rightLimit.end());
  if(rightLimit.size() > 0) obstacleInImage.right = rightLimit[rightLimit.size()/2];
}

void PlayersDeeptector::scanImage(std::vector<std::vector<Region> >& regions)
{
  std::vector<std::pair<int , int> >  yLimits(theECImage.grayscaled.width/xyStep);
  for(unsigned int x = 0, index = 0; index < theECImage.grayscaled.width/xyStep; x += xyStep, ++index)
    yLimits[index] = std::make_pair(theFieldBoundary.getBoundaryY(x) , theBodyContour.getBottom(x, theCameraInfo.height));

  const float yRegionsDivisor = static_cast<float>(theECImage.grayscaled.height) / xyRegions, xRegionsDivisor = static_cast<float>(theECImage.grayscaled.width) / xyRegions;

  const PixelTypes::GrayscaledPixel* pixUpperRow = theECImage.grayscaled[0];
  const PixelTypes::GrayscaledPixel* pixMidRow = theECImage.grayscaled[0];
  for(unsigned int y = 0; y < theECImage.grayscaled.height; y += xyStep)
  {
    const PixelTypes::GrayscaledPixel* pixLowerRow = y < theECImage.grayscaled.height-xyStep ? theECImage.grayscaled[y+xyStep] : pixMidRow;
    short leftSpot = *pixMidRow, midSpot = *pixMidRow;

    for(unsigned int x = 0, index = 0; x < theECImage.grayscaled.width; x += xyStep, ++index)
    {
      short rightSpot = x < theECImage.grayscaled.width-xyStep ? *(pixMidRow+x+xyStep) : midSpot;
      if(yLimits[index].first < static_cast<int>(y-xyStep) && yLimits[index].second > static_cast<int>(y+xyStep))
      {
        bool horizontalChange = std::abs(rightSpot-leftSpot) > ((rightSpot > brightnessThreshold || leftSpot > brightnessThreshold) ? minContrastDif/3*2 : minContrastDif);
        bool verticalChange = std::abs(*(pixLowerRow+x)-*(pixUpperRow+x)) > ((*(pixLowerRow+x) > brightnessThreshold || *(pixUpperRow+x) > brightnessThreshold) ? minContrastDif/3*2 : minContrastDif);
        Classification spotClassification = horizontalChange ? (verticalChange ? Mixed : Horizontal) : (verticalChange ? Vertical : Nothing);
        if(spotClassification != Nothing)
        {
          Region& region = regions[static_cast<int>(y / yRegionsDivisor)][static_cast<int>(x / xRegionsDivisor)];
          region.spots[spotClassification].emplace_back(Vector2i(x, y));
        }
        if(midSpot > brightnessThreshold)
          ++regions[static_cast<int>(y / yRegionsDivisor)][static_cast<int>(x / xRegionsDivisor)].brightSpots;
      }
      leftSpot = midSpot;
      midSpot = rightSpot;
    }
    pixUpperRow = pixMidRow;
    pixMidRow = pixLowerRow;
  }

  COMPLEX_DRAWING("module:PlayersDeeptector:spots")
  {
    std::vector<ColorRGBA> colors = {ColorRGBA::magenta, ColorRGBA::yellow, ColorRGBA::red};
    for(auto yLine : regions)
      for(Region& region : yLine)
      {
        for(unsigned int i = 0; i < region.spots.size(); ++i)
        {
          Vector2i offset = i == 0 ? Vector2i(xyStep, 0) : (i == 1 ? Vector2i(0, xyStep) : Vector2i(0, 0));
          for(Vector2i spot : region.spots[i])
            LINE("module:PlayersDeeptector:spots", spot.x(), spot.y(), (spot+offset).x(), (spot+offset).y(), 1, Drawings::solidPen, colors[i]);
        }
      }
  }
}

void PlayersDeeptector::classifyRegions(std::vector<std::vector<Region> >& regions)
{
  const int minSpotsToClassify = std::max(5, std::min(30, static_cast<int>((theECImage.grayscaled.width*theECImage.grayscaled.height) / (xyRegions*xyRegions) / (xyStep*xyStep) / 10)));

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      region.regionIndices = Vector2i(xRegionIndex, yRegionIndex);

      const float numOfHor = region.spots[0].size(), numOfVer = region.spots[1].size(), numOfWonky = region.spots[2].size();
      if(numOfHor+numOfVer >= minSpotsToClassify)
      {
        region.classification = (numOfHor > numOfVer ? numOfVer/(numOfVer+numOfHor) : numOfHor/(numOfVer+numOfHor)) > mixedThresh ? Mixed : (numOfHor > numOfVer ? Horizontal : Vertical);
        region.wonky = numOfWonky > (numOfWonky+numOfHor+numOfVer)/3.f;
      }
      else
        region.bright = region.brightSpots >= minSpotsToClassify;
    }

  COMPLEX_DRAWING("module:PlayersDeeptector:regions")
  {
    std::vector<ColorRGBA> colors = {ColorRGBA::red, ColorRGBA::blue, ColorRGBA::cyan, ColorRGBA::black, ColorRGBA::white};
    for(auto yLine : regions)
      for(Region region : yLine)
      {
        if(region.classification == Nothing && !region.bright)
          continue;
        Vector2i upperLeft(region.regionIndices.x() * (theECImage.grayscaled.width / xyRegions), region.regionIndices.y() * (theECImage.grayscaled.height / xyRegions));
        Vector2i lowerRight = Vector2i(upperLeft.x() + (theECImage.grayscaled.width / xyRegions), upperLeft.y() + (theECImage.grayscaled.height / xyRegions));
        if(region.wonky)
          RECTANGLE("module:PlayersDeeptector:regions", upperLeft.x()+3, upperLeft.y()+3, lowerRight.x()-3, lowerRight.y()-3, 1, Drawings::solidPen, ColorRGBA::yellow);
        if(region.bright)
          RECTANGLE("module:PlayersDeeptector:regions", upperLeft.x()+3, upperLeft.y()+3, lowerRight.x()-3, lowerRight.y()-3, 1, Drawings::solidPen, ColorRGBA::violet);
        RECTANGLE("module:PlayersDeeptector:regions", upperLeft.x()+1, upperLeft.y()+1, lowerRight.x()-1, lowerRight.y()-1, 1, Drawings::solidPen, colors[region.classification]);
      }
  }
}

void PlayersDeeptector::discardHomogenAreas(std::vector<std::vector<Region> >& regions)
{
  std::vector<Vector2i> toDiscard;
  std::vector<std::vector<bool> > visited(xyRegions, std::vector<bool>(xyRegions, false));

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region &region = regions[yRegionIndex][xRegionIndex];
      if(region.classification == Nothing || region.classification == Mixed || visited[yRegionIndex][xRegionIndex])
        continue;

      std::vector<Vector2i> toCheck = {Vector2i(xRegionIndex, yRegionIndex)};
      for(unsigned int index = 0; index < toCheck.size(); ++index)
      {
        Vector2i& toCheckIndices = toCheck[index];
        const int minXOffset = toCheckIndices.x() == 0 ? 0 : -1, maxXOffset = toCheckIndices.x() == static_cast<int>(xyRegions-1) ? 0 : 1;
        const int minYOffset = toCheckIndices.y() == 0 ? 0 : -1, maxYOffset = toCheckIndices.y() == static_cast<int>(xyRegions-1) ? 0 : 1;

        int nonHomogenCounter = 0;
        for(int i = minYOffset; i <= maxYOffset && nonHomogenCounter < minNonHomogenSpots; ++i)
          for(int j = minXOffset; j <= maxXOffset && nonHomogenCounter < minNonHomogenSpots; ++j)
          {
            Region& neighbor = regions[toCheckIndices.y() + i][toCheckIndices.x() + j];
            if(neighbor.classification != Nothing && neighbor.classification != region.classification)
              ++nonHomogenCounter;
          }

        if(nonHomogenCounter < minNonHomogenSpots)
        {
          toDiscard.push_back(toCheckIndices);
          visited[toCheckIndices.y()][toCheckIndices.x()] = true;
          std::vector<Vector2i> neighbors;
          regionQuery(regions, toCheckIndices, 1, false, neighbors, region.classification);
          for(Vector2i& neighborIndices : neighbors)
            if(!visited[neighborIndices.y()][neighborIndices.x()])
            {
              visited[neighborIndices.y()][neighborIndices.x()] = true;
              toCheck.push_back(neighborIndices);
              toDiscard.push_back(neighborIndices);
            }
        }
      }
    }
  for(Vector2i &discardIndices : toDiscard)
    regions[discardIndices.y()][discardIndices.x()].classification = Nothing;

  COMPLEX_DRAWING("module:PlayersDeeptector:reclassifiedRegions")
  {
    std::vector<ColorRGBA> colors = {ColorRGBA::red, ColorRGBA::blue, ColorRGBA::cyan, ColorRGBA::black, ColorRGBA::white};
    for(auto yLine : regions)
      for(Region region : yLine)
      {
        if(region.classification == Nothing && !region.bright)
          continue;
        Vector2i upperLeft(region.regionIndices.x() * (theECImage.grayscaled.width / xyRegions), region.regionIndices.y() * (theECImage.grayscaled.height / xyRegions));
        Vector2i lowerRight = Vector2i(upperLeft.x() + (theECImage.grayscaled.width / xyRegions), upperLeft.y() + (theECImage.grayscaled.height / xyRegions));
        if(region.wonky)
          RECTANGLE("module:PlayersDeeptector:reclassifiedRegions", upperLeft.x()+3, upperLeft.y()+3, lowerRight.x()-3, lowerRight.y()-3, 1, Drawings::solidPen, ColorRGBA::yellow);
        if(region.bright)
          RECTANGLE("module:PlayersDeeptector:reclassifiedRegions", upperLeft.x()+3, upperLeft.y()+3, lowerRight.x()-3, lowerRight.y()-3, 1, Drawings::solidPen, ColorRGBA::violet);
        RECTANGLE("module:PlayersDeeptector:reclassifiedRegions", upperLeft.x()+1, upperLeft.y()+1, lowerRight.x()-1, lowerRight.y()-1, 1, Drawings::solidPen, colors[region.classification]);
      }
  }
}

void PlayersDeeptector::dbscan(std::vector<std::vector<Region> >& regions, std::vector<Cluster>& clusters)
{
  for(std::vector<Region>& horizontalRegionLine : regions)
    for(Region& region : horizontalRegionLine)
    {
      if(region.classification == Nothing || region.label != Undefined)
        continue;
      std::vector<Vector2i> neighbors;
      regionQuery(regions, region.regionIndices, 1, true, neighbors);
      if(neighbors.size() < minNeighborPoints)
        region.label = Noise;
      else
      {
        Cluster cluster = {std::vector<Vector2i>(), Vector2f(0.f, 0.f)};
        if(expandCluster(regions, region, neighbors, cluster))
        {
          auto minMaxY = std::minmax_element(cluster.regions.begin(), cluster.regions.end(), [](Vector2i& a, Vector2i& b) { return a.y() < b.y();});
          auto minMaxX = std::minmax_element(cluster.regions.begin(), cluster.regions.end(), [](Vector2i& a, Vector2i& b) { return a.x() < b.x();});
          const float rWidth = std::min(cluster.massCenterOfRegions.x()-(*minMaxX.first).x(), (*minMaxX.second).x()-cluster.massCenterOfRegions.x());
          const float rHeight = std::min(cluster.massCenterOfRegions.y()-(*minMaxY.first).y(), (*minMaxY.second).y()-cluster.massCenterOfRegions.y());
          cluster.topLeft = Vector2i(std::floor(cluster.massCenterOfRegions.x()-rWidth), std::floor(cluster.massCenterOfRegions.y()-rHeight));
          cluster.bottomRight = Vector2i(std::ceil(cluster.massCenterOfRegions.x()+rWidth), std::ceil(cluster.massCenterOfRegions.y()+rHeight));
          clusters.emplace_back(cluster);
        }
      }
    }

  COMPLEX_DRAWING("module:PlayersDeeptector:clusters")
  {
    std::vector<ColorRGBA> colors = {ColorRGBA::red, ColorRGBA::blue, ColorRGBA::magenta, ColorRGBA::yellow, ColorRGBA::orange, ColorRGBA::violet, ColorRGBA::brown};
    int clusterNumber = 0;
    for(Cluster& cluster : clusters)
    {
      for(Vector2i& regionIndices : cluster.regions)
      {
        Vector2i upperLeft(regionIndices.x() * (theECImage.grayscaled.width / xyRegions), regionIndices.y() * (theECImage.grayscaled.height / xyRegions));
        Vector2i lowerRight = Vector2i(upperLeft.x() + (theECImage.grayscaled.width / xyRegions), upperLeft.y() + (theECImage.grayscaled.height / xyRegions));
        RECTANGLE("module:PlayersDeeptector:clusters", upperLeft.x()+1, upperLeft.y()+1, lowerRight.x()-1, lowerRight.y()-1, 1, Drawings::solidPen, colors[clusterNumber%colors.size()]);
        DRAWTEXT("module:PlayersDeeptector:clusters", ((upperLeft + lowerRight) / 2).x(), ((upperLeft + lowerRight) / 2).y(), 3, ColorRGBA::black, clusterNumber);
      }

      Vector2f massCenterOfRegions(cluster.massCenterOfRegions.x()*(static_cast<float>(theECImage.grayscaled.width)/xyRegions)+(static_cast<float>(theECImage.grayscaled.width)/xyRegions/2),
                                   cluster.massCenterOfRegions.y()*(static_cast<float>(theECImage.grayscaled.height)/xyRegions)+(static_cast<float>(theECImage.grayscaled.height)/xyRegions/2));
      CROSS("module:PlayersDeeptector:clusters", massCenterOfRegions.x(), massCenterOfRegions.y(), 5, 5, Drawings::solidPen, ColorRGBA::black);
      ++clusterNumber;
    }
  }
}

bool PlayersDeeptector::expandCluster(std::vector<std::vector<Region> >& regions, Region& region, std::vector<Vector2i>& neighbors, Cluster& cluster)
{
  int nonWonkyCounter = 0, mixedRegions = 0, nonMixedRegions = 0, brightRegions = 0;
  cluster.regions.emplace_back(region.regionIndices);
  if(!region.wonky)
  {
    cluster.massCenterOfRegions += region.regionIndices.cast<float>();
    ++nonWonkyCounter;
  }
  region.label = Clustered;
  region.classification == Mixed ? ++mixedRegions : ++nonMixedRegions;

  for(unsigned int i = 0; i < neighbors.size(); ++i)
  {
    Vector2i& neighborIndices = neighbors[i];
    Region& currentRegion = regions[neighborIndices.y()][neighborIndices.x()];
    if(currentRegion.label == Clustered)
      continue;
    if(!currentRegion.bright)
    {
      cluster.regions.emplace_back(neighborIndices);
      currentRegion.classification == Mixed ? ++mixedRegions : ++nonMixedRegions;
      if(!currentRegion.wonky)
      {
        cluster.massCenterOfRegions += currentRegion.regionIndices.cast<float>();
        ++nonWonkyCounter;
      }
    }
    else
      ++brightRegions;
    currentRegion.label = Clustered;

    std::vector<Vector2i> nextNeighbors;
    if(regionQuery(regions, neighborIndices, 1, true, nextNeighbors) && nextNeighbors.size() >= minNeighborPoints)
      for(Vector2i& nextNeighbor : nextNeighbors)
        if(regions[nextNeighbor.y()][nextNeighbor.x()].label != Clustered)
          neighbors.push_back(nextNeighbor);
  }
  cluster.massCenterOfRegions /= std::max(1, nonWonkyCounter);
  return nonMixedRegions > 0 && mixedRegions > 0 && (mixedRegions < 16 || static_cast<float>(nonMixedRegions)/static_cast<float>(mixedRegions) > 0.4f);
}

bool PlayersDeeptector::regionQuery(std::vector<std::vector<Region> >& regions, Vector2i& regionIndices, int dis, bool queryBright, std::vector<Vector2i>& neighbors, Classification classificationToSearch)
{
  int nonBright = 0;
  for(int i = -dis; i <= dis; ++i)
  {
    if(regionIndices.y() + i < 0 || regionIndices.y() + i >= static_cast<int>(xyRegions))
      continue;
    for(int j = -dis; j <= dis; ++j)
    {
      if(regionIndices.x() + j < 0 || regionIndices.x() + j >= static_cast<int>(xyRegions))
        continue;
      nonBright += regions[regionIndices.y() + i][regionIndices.x() + j].classification != Nothing ? 1 : 0;
      if((classificationToSearch == Unknown && regions[regionIndices.y() + i][regionIndices.x() + j].classification != Nothing) ||
         (classificationToSearch != Unknown && classificationToSearch == regions[regionIndices.y() + i][regionIndices.x() + j].classification) ||
         (queryBright && regions[regionIndices.y() + i][regionIndices.x() + j].bright))
        neighbors.emplace_back(Vector2i(regionIndices.x() + j, regionIndices.y() + i));
    }
  }
  return nonBright >= minNonHomogenSpots;
}

inline float cross(const Vector2i& o, const Vector2i& a, const Vector2i& b)
{
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

void PlayersDeeptector::calcConvexHulls(std::vector<Cluster>& clusters, std::vector<ObstaclesImagePercept::Obstacle>& obstacles, std::vector<std::vector<Region> >& regions)
{
  for(Cluster& cluster : clusters)
  {
    std::vector<Vector2i> clusterPoints;
    for(Vector2i& regionIndices : cluster.regions)
    {
      Region& region = regions[regionIndices.y()][regionIndices.x()];
      for(int i = 0; i < 3; ++i)
        clusterPoints.insert(clusterPoints.end(), region.spots[i].begin(), region.spots[i].end());
    }
    if(clusterPoints.empty())
      continue;

    ObstaclesImagePercept::Obstacle obstacle;
    int n = static_cast<int>(clusterPoints.size()), k = 0;
    obstacle.convexHull = std::vector<Vector2i>(2 * n);

    std::sort(clusterPoints.begin(), clusterPoints.end(), [](const Vector2i& p1, const Vector2i& p2) {return p1.x() < p2.x() || (p1.x() == p2.x() && p1.y() < p2.y()); });
    for(int i = 0; i < n; ++i)
    {
      while(k >= 2 && cross(obstacle.convexHull[k - 2], obstacle.convexHull[k - 1], clusterPoints[i]) <= 0)
        k--;
      obstacle.convexHull[k++] = clusterPoints[i];
    }
    for(int i = n - 2, t = k + 1; i >= 0; i--)
    {
      while(k >= t && cross(obstacle.convexHull[k - 2], obstacle.convexHull[k - 1], clusterPoints[i]) <= 0)
        k--;
      obstacle.convexHull[k++] = clusterPoints[i];
    }
    obstacle.convexHull.resize(k - 1);

    auto minMaxY = std::minmax_element(obstacle.convexHull.begin(), obstacle.convexHull.end(), [](Vector2i& a, Vector2i& b) { return a.y() < b.y();});
    auto minMaxX = std::minmax_element(obstacle.convexHull.begin(), obstacle.convexHull.end(), [](Vector2i& a, Vector2i& b) { return a.x() < b.x();});

    obstacle.top = (*minMaxY.first).y();
    for(int yRegionIndex = cluster.topLeft.y(); yRegionIndex >= 0 && regions[yRegionIndex][static_cast<int>(cluster.massCenterOfRegions.x())].bright; --yRegionIndex)
      obstacle.top -= theECImage.grayscaled.height / xyRegions;
    obstacle.bottom = (*minMaxY.second).y();
    obstacle.left = (*minMaxX.first).x();
    obstacle.right = (*minMaxX.second).x();
    obstacle.bottomFound = true;
    obstacle.fallen = false;
    if(trimObstacles)  trimObstacle(obstacle);

    if(obstacle.right-obstacle.left < minWidthInPixel || obstacle.bottom-obstacle.top < minWidthInPixel)
      continue;
    obstacles.emplace_back(obstacle);
  }
}


/**
 * Hue values for the different team colors. The indices are those of the TEAM_
 * constants defined in RoboCupGameControlData.h.
 */
static int jerseyHues[] =
{
  255 * 7 / 8, // blue + cyan
  255 * 2 / 8, // red
  255 * 4 / 8, // yellow
  0, // black (unused)
  0, // white (unused)
  0, // green (unsupported)
  255 * 3 / 8, // orange
  255 * 1 / 8, // purple
  255 * 7 / 16, // brown
  0, // gray (unused)
};

void PlayersDeeptector::detectJersey(const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField) const
{
  Vector2f lowerInImage;
  Vector2f upperInImage;
  float distance = obstacleOnField.center.norm() + theRobotDimensions.footLength * 0.5f;

  // Determine jersey region.
  const Vector2f centerOnField = obstacleOnField.center.normalized(distance);
  if(Transformation::robotToImage(Vector3f(centerOnField.x(), centerOnField.y(), jerseyYRange.min),
                                  theCameraMatrix, theCameraInfo, lowerInImage)
     && Transformation::robotToImage(Vector3f(centerOnField.x(), centerOnField.y(), jerseyYRange.max),
                                     theCameraMatrix, theCameraInfo, upperInImage))
  {
    lowerInImage = theImageCoordinateSystem.fromCorrected(lowerInImage);
    if(lowerInImage.y() >= jerseyYSamples)
    {
      upperInImage = theImageCoordinateSystem.fromCorrected(upperInImage);
      if(upperInImage.y() < 0)
        upperInImage = lowerInImage + (upperInImage - lowerInImage) * lowerInImage.y() / (lowerInImage.y() - upperInImage.y());
      if(lowerInImage.y() > static_cast<float>(theCameraInfo.height - 1))
        lowerInImage = upperInImage + (lowerInImage - upperInImage) * (static_cast<float>(theCameraInfo.height - 1) - upperInImage.y()) / (lowerInImage.y() - upperInImage.y());

      const int width = obstacleInImage.right - obstacleInImage.left + 1;
      const int ySteps = std::min(jerseyYSamples, static_cast<int>(lowerInImage.y() - upperInImage.y()));
      const float xStep = std::max(1.f, static_cast<float>(width) / static_cast<float>(jerseyXSamples));
      const float yStep = (lowerInImage.y() - upperInImage.y()) / static_cast<float>(ySteps);
      const float xyStep = (lowerInImage.x() - upperInImage.x()) / static_cast<float>(ySteps);

      // If gray is involved, the maximum brightness is determined, to determine gray ralative to it.
      unsigned char maxBrightness = 0;
      if(theOwnTeamInfo.teamColor == TEAM_GRAY || theOpponentTeamInfo.teamColor == TEAM_GRAY)
      {
        Vector2f centerInImage = (Vector2f(static_cast<float>(obstacleInImage.left + obstacleInImage.right) * 0.5f, static_cast<float>(obstacleInImage.bottom)) * (1.f - whiteScanHeightRatio)
                                  + lowerInImage * whiteScanHeightRatio);
        float left = std::max(centerInImage.x() - static_cast<float>(width) * 0.5f, 0.f);
        float right = std::min(centerInImage.x() + static_cast<float>(width) * 0.5f, static_cast<float>(theCameraInfo.width) - 0.5f);
        for(float x = left; x < right; x += xStep)
          maxBrightness = std::max(theECImage.grayscaled[static_cast<int>(centerInImage.y())][static_cast<int>(x)], maxBrightness);
      }

      // Get pixel classifier
      const std::function<bool(int, int)>& isOwn = getPixelClassifier(theOwnTeamInfo.teamColor, theOpponentTeamInfo.teamColor, maxBrightness);
      const std::function<bool(int, int)>& isOpponent = getPixelClassifier(theOpponentTeamInfo.teamColor, theOwnTeamInfo.teamColor, maxBrightness);

      int ownPixels = 0;
      int opponentPixels = 0;
      float left = std::max(upperInImage.x() - static_cast<float>(width) * 0.5f, 0.f);
      float right = std::min(upperInImage.x() + static_cast<float>(width) * 0.5f, static_cast<float>(theCameraInfo.width) - 0.5f);

      for(float y = upperInImage.y(); y < lowerInImage.y(); y += yStep, left = std::max(left + xyStep, 0.f), right = std::min(right + xyStep, static_cast<float>(theCameraInfo.width)))
        for(float x = left; x < right; x += xStep)
          if(isOwn(static_cast<int>(x), static_cast<int>(y)))
          {
            ++ownPixels;
            DOT("module:PlayersDeeptector:jersey", x, y, ColorRGBA::yellow, ColorRGBA::yellow);
          }
          else if(isOpponent(static_cast<int>(x), static_cast<int>(y)))
          {
            ++opponentPixels;
            DOT("module:PlayersDeeptector:jersey", x, y, ColorRGBA::blue, ColorRGBA::blue);
          }
          else
            DOT("module:PlayersDeeptector:jersey", x, y, ColorRGBA::green, ColorRGBA::green);

      if((ownPixels > minJerseyPixels || opponentPixels > minJerseyPixels)
         && (ownPixels > (ownPixels + opponentPixels) * minJerseyRatio
             || opponentPixels > (ownPixels + opponentPixels) * minJerseyRatio))
        obstacleOnField.type = ownPixels > opponentPixels ? ObstaclesFieldPercept::ownPlayer : ObstaclesFieldPercept::opponentPlayer;
    }
  }
}

std::function<bool(int, int)> PlayersDeeptector::getPixelClassifier(const int teamColor, const int otherColor, const int maxBrightness) const
{
  const int teamHue = jerseyHues[teamColor];
  const int otherHue = jerseyHues[otherColor];
  const Rangei grayRange = Rangei(static_cast<int>(maxBrightness * this->grayRange.min),
                                  static_cast<int>(maxBrightness * this->grayRange.max));

  // If gray is involved, different brightnesses must be distinguished.
  if(teamColor == TEAM_GRAY)
    return [this, grayRange](const int x, const int y)
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      return color != FieldColors::none && color != FieldColors::field && grayRange.isInside(theECImage.grayscaled[y][x]);
    };
  else if(teamColor == TEAM_BLACK && otherColor == TEAM_GRAY)
    return [this, grayRange](const int x, const int y)
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      return color != FieldColors::none && color != FieldColors::field && theECImage.grayscaled[y][x] < grayRange.min;
    };
  else if(teamColor == TEAM_WHITE && otherColor == TEAM_GRAY)
    return [this, grayRange](const int x, const int y)
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      return color != FieldColors::none && color != FieldColors::field && theECImage.grayscaled[y][x] > grayRange.max;
    };

    // Black and white can be determined directly
  else if(teamColor == TEAM_BLACK)
    return [this](const int x, const int y) {return theECImage.colored[y][x] == FieldColors::black;};
  else if(teamColor == TEAM_WHITE)
    return [this](const int x, const int y) {return theECImage.colored[y][x] == FieldColors::white;};

    // This team uses a color, the other team does not.
  else if(otherColor == TEAM_WHITE || otherColor == TEAM_BLACK || otherColor == TEAM_GRAY)
    return [this, teamHue](const int x, const int y)
    {
      return theECImage.colored[y][x] == FieldColors::none
             && std::abs(static_cast<char>(theECImage.hued[y][x] - teamHue)) <= hueSimilarityThreshold;
    };

    // Both teams use colors, use the closer one.
  else
    return [this, teamHue, otherHue](const int x, const int y)
    {
      if(theECImage.colored[y][x] == FieldColors::none)
      {
        const int hue = theECImage.hued[y][x];
        // Casting to char should resolve the wraparound cases.
        return std::abs(static_cast<char>(hue - teamHue)) < std::abs(static_cast<char>(hue - otherHue));
      }
      else
        return false;
    };
}
