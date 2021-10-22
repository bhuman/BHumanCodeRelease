/**
 * @file PlayersDeeptector.cpp
 *
 * This file implements a module that detects players in images with a neural network.
 *
 * @author Bernd Poppinga
 */

#include "PlayersDeeptector.h"
#include "Platform/File.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Global.h"
#include "Tools/ImageProcessing/PatchUtilities.h"
#include "Tools/ImageProcessing/Resize.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include <CompiledNN/Model.h>

MAKE_MODULE(PlayersDeeptector, perception);

PlayersDeeptector::PlayersDeeptector() :
  convModel(&Global::getAsmjitRuntime())
{
  NeuralNetwork::CompilationSettings settings;
  settings.useExpApproxInSigmoid = false;
  settings.useExpApproxInTanh = false;

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    model = std::make_unique<NeuralNetwork::Model>(std::string(File::getBHDir()) + "/Config/NeuralNets/PlayersDeeptector/players_deeptector.h5");
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
}

void PlayersDeeptector::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
  theObstaclesImagePercept.obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
}

void PlayersDeeptector::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{
  DECLARE_DEBUG_DRAWING("module:PlayersDeeptector:image", "drawingOnImage");

  std::vector<ObstaclesImagePercept::Obstacle>& obstacles = theCameraInfo.camera == CameraInfo::upper ? obstaclesUpper : obstaclesLower;
  obstacles.clear();
  theObstaclesFieldPercept.obstacles.clear();
  const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.clear();

  if(theCameraInfo.camera == CameraInfo::upper)
  {
    LabelImage labelImage;
    if(!convModel.valid() || !theFieldBoundary.isValid || !theECImage.grayscaled.width || !theECImage.grayscaled.height)
      return;

    const unsigned int scale = static_cast<unsigned int>(std::log2(theECImage.grayscaled.width / patchSize(0)) + 0.5);
    ASSERT(theECImage.grayscaled.width == static_cast<unsigned>(patchSize(0)) << scale);
    ASSERT(theECImage.grayscaled.height == static_cast<unsigned>(patchSize(1)) << scale);

    // Can't shrink directly to NN input because it needs a larger buffer :-(
    STOPWATCH("module:PlayersDeeptector:shrinkY") Resize::shrinkY(scale, theECImage.grayscaled, thumbnail);
    ASSERT(patchSize(0) == static_cast<int>(thumbnail.width));
    ASSERT(patchSize(1) == static_cast<int>(thumbnail.height));
    std::memcpy(reinterpret_cast<unsigned char*>(convModel.input(0).data()), thumbnail[0], thumbnail.width * thumbnail.height * sizeof(unsigned char));
    STOPWATCH("module:PlayersDeeptector:normalizeContrast")
      PatchUtilities::normalizeContrast<unsigned char>(reinterpret_cast<unsigned char*>(convModel.input(0).data()), patchSize, 0.02f);
    STOPWATCH("module:PlayersDeeptector:apply") convModel.apply();
    STOPWATCH("module:PlayersDeeptector:boundingBoxes") boundingBoxes(labelImage);

    STOPWATCH("module:PlayersDeeptector:nonMaximumSuppression") labelImage.nonMaximumSuppression(0.3f);
    STOPWATCH("module:PlayersDeeptector:bigBoxSuppression") labelImage.bigBoxSuppression();

    for(const LabelImage::Annotation& box : labelImage.annotations)
    {
      DRAW_TEXT("module:PlayersDeeptector:image", box.upperLeft.x(), box.upperLeft.y() - 2, 10, ColorRGBA::black, "dist: " << box.distance << "m");
      DRAW_TEXT("module:PlayersDeeptector:image", box.upperLeft.x(), box.upperLeft.y() - 12, 10, ColorRGBA::black, "prob: " << box.probability);
      obstacles.emplace_back(ObstaclesImagePercept::Obstacle());
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

    std::vector<std::vector<Region>> regions(xyRegions, std::vector<Region>(xyRegions, Region()));
    STOPWATCH("module:PlayersDeeptector:scanImage") scanImage(regions);
    STOPWATCH("module:PlayersDeeptector:classifyRegions") classifyRegions(regions);
    STOPWATCH("module:PlayersDeeptector:discardHomogenAreas") discardHomogeneousAreas(regions);
    STOPWATCH("module:PlayersDeeptector:clusterRegions") dbScan(regions, obstacles);
  }

  bool mergeObstacles = false;
  do
  {
    mergeObstacles = mergeLowerObstacles && theCameraInfo.camera == CameraInfo::lower && !mergeObstacles && obstacles.size() >= 2;
    auto it = obstacles.begin();
    while(it != obstacles.end())
    {
      bool validObstacle;
      ObstaclesImagePercept::Obstacle& obstacleInImage = *it;
      ObstaclesFieldPercept::Obstacle obstacleOnField;

      if((validObstacle = (Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.left) &&
                           Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.right))))
      {
        obstacleOnField.fallen = obstacleInImage.fallen;
        obstacleOnField.type = ObstaclesFieldPercept::unknown;
        obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

        const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
        if(theCameraInfo.camera == CameraInfo::upper)
        {
          RECTANGLE("module:PlayersDeeptector:image", obstacleInImage.left, obstacleInImage.top, obstacleInImage.right, obstacleInImage.bottom, 3, Drawings::solidPen, ColorRGBA::violet);
          if(obstacleInImage.bottom > theECImage.grayscaled.height * 0.9f)
            for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
            {
              const Rangea rangeLower((theOdometer.odometryOffset.inverse() * incompleteObstacle.right).angle(), (theOdometer.odometryOffset.inverse() * incompleteObstacle.left).angle());
              if(range.min <= rangeLower.max && rangeLower.min <= range.max)
                obstacleInImage.bottomFound = false;
            }

          if(!trimObstacles || (validObstacle = trimObstacle(false, obstacleInImage) &&
                                                Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.left) &&
                                                Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)), theCameraMatrix, theCameraInfo, obstacleOnField.right) &&
                                                (obstacleOnField.right - obstacleOnField.left).squaredNorm() <= sqr(750.f)))
          {
            if(!obstacleInImage.bottomFound)
            {
              STOPWATCH("module:PlayersDeeptector:detectJersey") theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
              const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
            }
            else if((validObstacle = obstacleInImage.right - obstacleInImage.left < static_cast<int>(theECImage.grayscaled.width / 2)))
            {
              obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;
              STOPWATCH("module:PlayersDeeptector:detectJersey") theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
              theObstaclesFieldPercept.obstacles.emplace_back(obstacleOnField);
            }
          }
        }
        else if((validObstacle = obstacleInImage.top < static_cast<int>(theECImage.grayscaled.height / xyRegions) || static_cast<float>(obstacleInImage.bottom - obstacleInImage.top) / theECImage.grayscaled.height > 0.5f))
        {
          if(!mergeObstacles)
            STOPWATCH("module:PlayersDeeptector:detectJersey")
              theJerseyClassifier.detectJersey(obstacleInImage, obstacleOnField);
          float minWidth = minWidthOnFieldNoMatch;
          bool hasMatch = false;
          for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : theOtherObstaclesPerceptorData.incompleteObstacles)
          {
            const Rangea rangeUpper((theOdometer.odometryOffset.inverse() * incompleteObstacle.right).angle(), (theOdometer.odometryOffset.inverse() * incompleteObstacle.left).angle());
            if((hasMatch = (range.min <= rangeUpper.max && rangeUpper.min <= range.max)))
            {
              obstacleOnField.type = incompleteObstacle.type;
              minWidth = minWidthOnFieldNoMatch / 2;
              break;
            }
          }
          obstacleOnField.fallen = !hasMatch && (obstacleInImage.right - obstacleInImage.left >= static_cast<int>(theECImage.grayscaled.width / 2) ||
                                                 obstacleInImage.bottom - obstacleInImage.top >= static_cast<int>(theECImage.grayscaled.height / 2));

          RECTANGLE("module:PlayersDeeptector:image", obstacleInImage.left, obstacleInImage.top, obstacleInImage.right, obstacleInImage.bottom, 3, Drawings::solidPen, ColorRGBA::violet);
          if((validObstacle = (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= sqr(minWidth)) && !mergeObstacles)
          {
            const_cast<ObstaclesPerceptorData&>(theObstaclesPerceptorData).incompleteObstacles.emplace_back(obstacleOnField);
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

void PlayersDeeptector::update(ObstaclesPerceptorData& theObstaclesPerceptorData)
{
  theObstaclesPerceptorData.cameraInfo = theCameraInfo;
  theObstaclesPerceptorData.cameraMatrix = theCameraMatrix;
  theObstaclesPerceptorData.imageCoordinateSystem = theImageCoordinateSystem;
}

void PlayersDeeptector::boundingBoxes(LabelImage& labelImage)
{
  const float threshold = -std::log(1.f / objectThres - 1);
  for(unsigned y = 0; y < convModel.output(0).dims(0); ++y)
    for(unsigned x = 0; x < convModel.output(0).dims(1); ++x)
      for(unsigned b = 0; b < 4; ++b)
      {
        const size_t offset = (y * convModel.output(0).dims(1) + x) * 4 * 6;
        if(convModel.output(0)[offset + b * 6 + 4] > threshold)
        {
          Eigen::Map<Eigen::Matrix<float, 4, 6, Eigen::RowMajor>> pred(convModel.output(0).data() + offset);
          pred.array() = 1.f / (1.f + (pred * -1).array().exp());

          pred.col(0) = (x + pred.col(0).array()).matrix() / convModel.output(0).dims(1) * theCameraInfo.width;
          pred.col(1) = (y + pred.col(1).array()).matrix() / convModel.output(0).dims(0) * theCameraInfo.height;
          pred.col(2).array() *= 10 * anchors.col(0).array() / convModel.output(0).dims(1) * theCameraInfo.width;
          pred.col(3).array() *= 10 * anchors.col(1).array() / convModel.output(0).dims(0) * theCameraInfo.height;
          pred.col(5).array() *= 10;

          LabelImage::Annotation box;
          box.upperLeft = Vector2f(pred(b, 0) - pred(b, 2) / 2, pred(b, 1) - pred(b, 3) / 2);
          box.lowerRight = Vector2f(pred(b, 0) + pred(b, 2) / 2, pred(b, 1) + pred(b, 3) / 2);
          box.probability = pred(b, 4);
          box.distance = pred(b, 5);
          box.measuredDistance = box.distance;

          Vector2f point = Vector2f((box.lowerRight.x() + box.upperLeft.x()) / 2, box.lowerRight.y());
          if(box.lowerRight.y() < theCameraInfo.height - 50 && Transformation::imageToRobot((Vector2i)point.cast<int>(), theCameraMatrix, theCameraInfo, point))
            box.measuredDistance = point.norm() / 1000;
          if(theFieldBoundary.getBoundaryY(static_cast<int>((box.lowerRight.x() + box.upperLeft.x()) / 2.f)) > box.lowerRight.y())
            continue;
          labelImage.annotations.emplace_back(box);
        }
      }
}

bool PlayersDeeptector::trimObstacle(bool trimHeight, ObstaclesImagePercept::Obstacle& obstacleInImage)
{
  const int minX = std::max(obstacleInImage.left, 0);
  const int minY = theCameraInfo.camera == CameraInfo::lower ? std::max(obstacleInImage.top, 0) : std::max(static_cast<int>(obstacleInImage.top + (obstacleInImage.bottom - obstacleInImage.top) * 0.5f), 0);
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
        if(std::abs(*secondSpot - firstSpot) > minContrastDif / 2)
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

void PlayersDeeptector::scanImage(std::vector<std::vector<Region>>& regions)
{
  std::vector<std::pair<int, int>> yLimits(theECImage.grayscaled.width / xyStep);
  for(unsigned int x = 0, index = 0; index < theECImage.grayscaled.width / xyStep; x += xyStep, ++index)
    yLimits[index] = std::make_pair(theFieldBoundary.getBoundaryY(x), theBodyContour.getBottom(x, theCameraInfo.height));
  const float yRegionsDivisor = static_cast<float>(theECImage.grayscaled.height) / xyRegions, xRegionsDivisor = static_cast<float>(theECImage.grayscaled.width) / xyRegions;

  std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> upperRow = {theECImage.grayscaled[0], theECImage.saturated[0]};
  std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> midRow = upperRow;
  std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> lowerRow;
  for(unsigned int y = 0; y < theECImage.grayscaled.height - xyStep; y += xyStep, upperRow = midRow, midRow = lowerRow)
  {
    lowerRow = {theECImage.grayscaled[y + xyStep], theECImage.saturated[y + xyStep]};
    short leftLum = *midRow.first, leftSat = *midRow.second, midLum = leftLum, midSat = leftSat;
    const PixelTypes::GrayscaledPixel* rightLum = midRow.first;
    const PixelTypes::GrayscaledPixel* rightSat = midRow.second;

    for(unsigned int x = 0, index = 0; x < theECImage.grayscaled.width - xyStep; x += xyStep, ++index, leftLum = midLum, midLum = *rightLum, leftSat = midSat, midSat = *rightSat)
    {
      rightLum += xyStep;
      rightSat += xyStep;
      if(yLimits[index].first < static_cast<int>(y - xyStep) && yLimits[index].second > static_cast<int>(y + xyStep))
      {
        bool horizontalChange = (leftSat < satThreshold || *rightSat < satThreshold) && std::abs(*rightLum - leftLum) > minContrastDif;
        bool verticalChange = (*(upperRow.second + x) < satThreshold || *(lowerRow.second + x) < satThreshold) &&
                              std::abs(static_cast<short>(*(upperRow.first + x)) - static_cast<short>(*(lowerRow.first + x))) > minContrastDif;
        Classification spotClassification = horizontalChange ? (verticalChange ? Nothing : Horizontal) : (verticalChange ? Vertical : Nothing);
        if(spotClassification != Nothing)
        {
          Region& region = regions[static_cast<int>(y / yRegionsDivisor)][static_cast<int>(x / xRegionsDivisor)];
          ++region.contrastChanges[spotClassification];
          if(static_cast<int>(y) > region.maxY) region.maxY = y;
        }
        if(midSat < satThreshold && midLum >= brightnessThreshold)
          ++regions[static_cast<int>(y / yRegionsDivisor)][static_cast<int>(x / xRegionsDivisor)].brightSpots;
      }
    }
  }
}

void PlayersDeeptector::classifyRegions(std::vector<std::vector<Region>>& regions)
{
  const int minSpotsToClassify = std::max(5, std::min(30, static_cast<int>((theECImage.grayscaled.width * theECImage.grayscaled.height) / (xyRegions * xyRegions) / (xyStep * xyStep) / 10)));

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      region.regionIndices = Vector2i(xRegionIndex, yRegionIndex);

      const float hor = static_cast<float>(region.contrastChanges[0]);
      const float ver = static_cast<float>(region.contrastChanges[1]);
      if(hor + ver >= minSpotsToClassify)
        region.classification = (hor > ver ? ver / (ver + hor) : hor / (ver + hor)) > mixedThresh ? Mixed : (hor > ver ? Horizontal : Vertical);
      region.bright = region.brightSpots >= minSpotsToClassify;
    }
}

void PlayersDeeptector::discardHomogeneousAreas(std::vector<std::vector<Region>>& regions)
{
  std::vector<Vector2i> toDiscard;
  std::vector<std::vector<bool>> visited(xyRegions, std::vector<bool>(xyRegions, false));

  auto checkHomogeneity = [this](std::vector<std::vector<Region>>& regions, Vector2i& indices, Classification relevantClass) -> bool
  {
    int hetCounter = 0;
    for(int y = std::max(0, indices.y() - 1); hetCounter < minHetSpots && y <= std::min(static_cast<int>(xyRegions - 1), indices.y() + 1); ++y)
      for(int x = std::max(0, indices.x() - 1); hetCounter < minHetSpots && x <= std::min(static_cast<int>(xyRegions - 1), indices.x() + 1); ++x)
      {
        const Region& neighbor = regions[y][x];
        if(neighbor.classification != Nothing&& neighbor.classification != relevantClass)
          ++hetCounter;
        if(hetCounter >= minHetSpots)
          return false;
      }
    return true;
  };

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      if(region.classification == Nothing || region.classification == Mixed || visited[yRegionIndex][xRegionIndex])
        continue;

      std::vector<Vector2i> toCheck = {Vector2i(xRegionIndex, yRegionIndex)};
      for(size_t index = 0; index < toCheck.size(); ++index)
      {
        Vector2i& indices = toCheck[index];
        if(checkHomogeneity(regions, indices, region.classification))
        {
          toDiscard.emplace_back(indices);
          visited[indices.y()][indices.x()] = true;
          std::vector<Vector2i> neighbors;
          regionQuery(regions, indices, 1, neighbors, region.classification);
          for(Vector2i& neighborIndices : neighbors)
            if(!visited[neighborIndices.y()][neighborIndices.x()])
            {
              visited[neighborIndices.y()][neighborIndices.x()] = true;
              toCheck.emplace_back(neighborIndices);
              toDiscard.emplace_back(neighborIndices);
            }
        }
      }
    }
  for(Vector2i& discardIndices : toDiscard)
    regions[discardIndices.y()][discardIndices.x()].classification = Nothing;

  for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex)
    for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex)
    {
      Region& region = regions[yRegionIndex][xRegionIndex];
      if(region.classification == Nothing && region.bright && !checkHomogeneity(regions, region.regionIndices, Unknown))
        region.classification = Unknown;
    }
}

void PlayersDeeptector::dbScan(std::vector<std::vector<Region>>& regions, std::vector<ObstaclesImagePercept::Obstacle>& obstacles)
{
  for(std::vector<Region>& horizontalRegionLine : regions)
    for(Region& region : horizontalRegionLine)
    {
      if(region.classification == Nothing || region.clustered)
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
          obstacle.top = topLeft.y() * (theECImage.grayscaled.height / xyRegions);
          obstacle.bottom = bottomRight.y();
          obstacle.left = topLeft.x() * (theECImage.grayscaled.width / xyRegions);;
          obstacle.right = (bottomRight.x() + 1) * (theECImage.grayscaled.width / xyRegions);;
          obstacle.bottomFound = true;
          obstacle.fallen = false;

          if(trimObstacles && trimObstacle(true, obstacle))
            obstacles.emplace_back(obstacle);
        }
      }
    }
}

bool PlayersDeeptector::expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight)
{
  bool containsMixedRegions = false;
  cluster.emplace_back(region.regionIndices);
  region.clustered = true;
  containsMixedRegions = containsMixedRegions || region.classification == Mixed;

  for(unsigned int i = 0; i < neighbors.size(); ++i)
  {
    Vector2i& neighborIndices = neighbors[i];
    Region& currentRegion = regions[neighborIndices.y()][neighborIndices.x()];
    if(currentRegion.clustered)
      continue;

    cluster.emplace_back(neighborIndices);
    containsMixedRegions = containsMixedRegions || currentRegion.classification == Mixed;
    currentRegion.clustered = true;
    if(currentRegion.classification != Unknown)
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
      if(regions[yRegionIndex][xRegionIndex].bright)
      {
        cluster.emplace_back(Vector2i(xRegionIndex, yRegionIndex));
        topLeft = Vector2i(topLeft.x(), yRegionIndex);
      }
  return containsMixedRegions;
}

void PlayersDeeptector::regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& regionIndices, int dis, std::vector<Vector2i>& neighbors, Classification classificationToSearch)
{
  for(int y = std::max(regionIndices.y() - dis, 0); y < std::min(regionIndices.y() + dis + 1, static_cast<int>(xyRegions)); ++y)
    for(int x = std::max(regionIndices.x() - dis, 0); x < std::min(regionIndices.x() + dis + 1, static_cast<int>(xyRegions)); ++x)
      if((classificationToSearch == Default && regions[y][x].classification != Nothing) || (classificationToSearch == regions[y][x].classification))
        neighbors.emplace_back(Vector2i(x, y));
}
