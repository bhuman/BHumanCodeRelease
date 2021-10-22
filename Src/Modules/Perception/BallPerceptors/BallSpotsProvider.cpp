/**
 * @file BallSpotsProvider.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "BallSpotsProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Math/Transformation.h"

#include <cmath>

void BallSpotsProvider::update(BallSpots& ballSpots)
{
  DECLARE_DEBUG_DRAWING("module:BallSpotsProvider:scanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallSpotsProvider:searchScanLines", "drawingOnImage");

  ballSpots.ballSpots.clear();

  // Add a prediction based on the previous ball model to the candidates
  ballSpots.firstSpotIsPredicted = false;
  Vector2f predictionInImage;
  if(theFrameInfo.getTimeSince(theWorldModelPrediction.timeWhenBallLastSeen) < 100
     && Transformation::robotToImage(Vector3f(theWorldModelPrediction.ballPosition.x(), theWorldModelPrediction.ballPosition.y(), theBallSpecification.radius), theCameraMatrix, theCameraInfo, predictionInImage))
  {
    predictionInImage = theImageCoordinateSystem.fromCorrected(predictionInImage);
    const int x = static_cast<int>(std::round(predictionInImage.x())), y = static_cast<int>(std::round(predictionInImage.y()));

    if(x >= 0 && x < theCameraInfo.width && y >= 0 && y < theCameraInfo.height)
    {
      ballSpots.firstSpotIsPredicted = true;
      ballSpots.addBallSpot(x, y);
    }
  }

  searchScanLines(ballSpots);
}

void BallSpotsProvider::searchScanLines(BallSpots& ballSpots) const
{
  const unsigned step = theColorScanLineRegionsVerticalClipped.lowResStep > 1 ? theColorScanLineRegionsVerticalClipped.lowResStep / 2 : 1;
  const unsigned start = theColorScanLineRegionsVerticalClipped.lowResStart >= step ? theColorScanLineRegionsVerticalClipped.lowResStart - step : theColorScanLineRegionsVerticalClipped.lowResStart;
  Geometry::Circle circle;

  for(unsigned scanLineIndex = start; scanLineIndex < theColorScanLineRegionsVerticalClipped.scanLines.size(); scanLineIndex += step)
  {
    int lowestYOfCurrentArea = 0;
    int currentLengthNeeded = 0;
    for(const ScanLineRegion& region : theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions)
    {
      if(region.color != PixelTypes::Color::field)
      {
        if(currentLengthNeeded == 0)
        {
          lowestYOfCurrentArea = region.range.lower;
          currentLengthNeeded = static_cast<int>(getNeededLengthFor(theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, region.range.lower, circle));
          if(!currentLengthNeeded)
            break;
        }
      }
      else
      {
        if(currentLengthNeeded != 0)
        {
          if(lowestYOfCurrentArea - region.range.lower > currentLengthNeeded)
            ballSpots.ballSpots.emplace_back(circle.center.cast<int>());
          else if(lowestYOfCurrentArea == theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions.front().range.lower &&
                  lowestYOfCurrentArea - region.range.lower > currentLengthNeeded / 2)
            ballSpots.ballSpots.emplace_back(theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, (lowestYOfCurrentArea + region.range.lower) / 2);
          else
            goto noSpot;

          if(!(isLastDuplicative(ballSpots, static_cast<int>(circle.radius * minAllowedDistanceRadiusRelation))
               || isSpotClearlyInsideARobot(ballSpots.ballSpots.back(), circle.radius)))
          {
            unsigned char luminanceRef = 0;
            unsigned char saturationRef = 0;
            int luminanceAverage = 0;
            int saturationAverage = 0;
            for(int i = region.range.lower; i < lowestYOfCurrentArea; i++)
            {
              const unsigned char luminance = theECImage.grayscaled[i][theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x];
              const unsigned char saturation = theECImage.saturated[i][theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x];
              luminanceAverage += luminance;
              saturationAverage += saturation;
              if(luminance > luminanceRef)
              {
                luminanceRef = luminance;
              }
              if(saturation > saturationRef)
              {
                saturationRef = saturation;
              }
            }
            luminanceRef = static_cast<unsigned char>((static_cast<int>(luminanceAverage / (lowestYOfCurrentArea - region.range.lower)) + (lessStrictChecks ? 0.f : luminanceRef)) / 2);
            saturationRef = static_cast<unsigned char>((static_cast<int>(luminanceAverage / (lowestYOfCurrentArea - region.range.lower)) + (lessStrictChecks ? 0.f : saturationRef)) / 2);
            if(!correctWithScanLeftAndRight(ballSpots.ballSpots.back(), circle, luminanceRef, saturationRef)
               || (currentLengthNeeded < minRadiusOfWantedRegion
                   && !checkGreenAround(ballSpots.ballSpots.back(), circle.radius, luminanceRef, saturationRef))
               || isSpotClearlyInsideARobot(ballSpots.ballSpots.back(), circle.radius))
            {
              ballSpots.ballSpots.pop_back();
            }
          }
          else
          {
            ballSpots.ballSpots.pop_back();
          }
        }

      noSpot:
        lowestYOfCurrentArea = 0;
        currentLengthNeeded = 0;
      }
    }

    if(allowScanLineTopSpotFitting
       && currentLengthNeeded > sqr(minRadiusOfWantedRegion)
       && lowestYOfCurrentArea - theColorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions.back().range.upper > std::sqrt(currentLengthNeeded)
       && !isSpotClearlyInsideARobot(circle.center.cast<int>(), circle.radius))
    {
      ballSpots.ballSpots.emplace_back(circle.center.cast<int>());

      if(isLastDuplicative(ballSpots, static_cast<int>(circle.radius * minAllowedDistanceRadiusRelation)))
        ballSpots.ballSpots.pop_back();
    }
  }
}

bool BallSpotsProvider::correctWithScanLeftAndRight(Vector2i& initialPoint, const Geometry::Circle& circle, unsigned char luminanceRef, unsigned char saturationRef) const
{
  const int maxScanLength = static_cast<int>(circle.radius * scanLengthRadiusFactor);

  int leftMaximum(0), rightMaximum(theECImage.grayscaled.width);
  theBodyContour.clipLeft(leftMaximum, initialPoint.y());
  theBodyContour.clipRight(rightMaximum, initialPoint.y());

  const int maxLeftScanLength = std::min(maxScanLength, initialPoint.x() - leftMaximum);
  const int maxRightScanLength = std::min(maxScanLength, rightMaximum - initialPoint.x());

  unsigned foundGoodPixel = 0;
  int leftScanLength = 0;
  scanBallSpotOneDirection(initialPoint, leftScanLength, maxLeftScanLength, foundGoodPixel,
  [](const Vector2i& spot, const int currentLength) {return int(spot.x() - currentLength); },
  [](const Vector2i& spot, const int) {return int(spot.y()); },
  luminanceRef, saturationRef);

  int rightScanLength = 0;
  scanBallSpotOneDirection(initialPoint, rightScanLength, maxRightScanLength, foundGoodPixel,
  [](const Vector2i& spot, const int currentLength) {return int(spot.x() + currentLength); },
  [](const Vector2i& spot, const int) {return int(spot.y()); },
  luminanceRef, saturationRef);

  initialPoint.x() -= (leftScanLength - rightScanLength) / 2;
  const float noise = 1.f - static_cast<float>(foundGoodPixel) / static_cast<float>(leftScanLength + rightScanLength);
  const float foundDiameterPercentage = static_cast<float>(leftScanLength + rightScanLength) / static_cast<float>(2 * circle.radius);

  DRAW_TEXT("module:BallSpotsProvider:scanLines", initialPoint.x() + 10, initialPoint.y() - 4, 6, ColorRGBA::red, "noise:" << noise);
  DRAW_TEXT("module:BallSpotsProvider:scanLines", initialPoint.x() + 10, initialPoint.y() + 5, 6, ColorRGBA::red, "foundDiameterPercentage:" << foundDiameterPercentage);
  return noise < noiseThreshold && foundDiameterPercentage > minFoundDiameterPercentage;
}

void BallSpotsProvider::scanBallSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
                                                       unsigned& goodPixelCounter,
                                                       int(*getX)(const Vector2i& spot, const int currentLength),
                                                       int(*getY)(const Vector2i& spot, const int currentLength),
                                                       unsigned char luminanceRef, unsigned char saturationRef) const
{
  unsigned currentSkipped = 0;
  while(checkPixel(theECImage.grayscaled[getY(spot, currentLength)][getX(spot, currentLength)], theECImage.saturated[getY(spot, currentLength)][getX(spot, currentLength)], goodPixelCounter, currentSkipped, luminanceRef, saturationRef)
        && ++currentLength <= maxLength);
  currentLength -= currentSkipped;

  LINE("module:BallSpotsProvider:scanLines", spot.x(), spot.y(), getX(spot, maxLength), getY(spot, maxLength),
       1, Drawings::solidPen, ColorRGBA::yellow);
  LINE("module:BallSpotsProvider:scanLines", spot.x(), spot.y(), getX(spot, currentLength), getY(spot, currentLength),
       1, Drawings::solidPen, ColorRGBA::red);
}

bool BallSpotsProvider::checkPixel(unsigned char pixelLuminance, unsigned char pixelSaturation, unsigned& goodPixelCounter, unsigned& currentSkipped, unsigned char luminanceRef, unsigned char saturationRef) const
{
  if(!theRelativeFieldColors.isFieldNearWhite(pixelLuminance, pixelSaturation, luminanceRef, saturationRef))
  {
    currentSkipped = 0;
    ++goodPixelCounter;
  }
  else
    ++currentSkipped;
  return currentSkipped < maxNumberOfSkippablePixel;
}

bool BallSpotsProvider::isLastDuplicative(const BallSpots& ballSpots, const int minAllowedDistance) const
{
  if(ballSpots.ballSpots.size() < 2)
    return false;

  const int squaredAllowedDistance = sqr(minAllowedDistance);
  const Vector2i& spotToCheck = ballSpots.ballSpots.back();
  for(auto ptr = ballSpots.ballSpots.begin(); ptr < ballSpots.ballSpots.end() - 1; ptr++)
    if((*ptr - spotToCheck).squaredNorm() < squaredAllowedDistance)
      return true;

  return false;
}

bool BallSpotsProvider::checkGreenAround(const Vector2i& spot, const float radius, unsigned char luminanceRef, unsigned char saturationRef) const
{
  CIRCLE("module:BallSpotsProvider:searchScanLines", static_cast<int>(spot.x()), static_cast<int>(spot.y()), static_cast<int>(radius), 2, Drawings::PenStyle::solidPen, ColorRGBA::red, Drawings::BrushStyle::solidBrush, ColorRGBA::red);
  int useRadius = additionalRadiusForGreenCheck + static_cast<int>(radius);
  if(useRadius >= spot.x() - 1 || useRadius >= spot.y() - 1 ||
     spot.x() + 1 + useRadius >= static_cast<int>(theECImage.grayscaled.width) ||
     spot.y() + 1 + useRadius >= static_cast<int>(theECImage.grayscaled.height))
    return false;
  int count(0);
  const int lastX = spot.x() + useRadius - 1;

  const int y1 = spot.y() - useRadius;
  const int y12 = spot.y() - useRadius - 1;
  const int y2 = spot.y() + useRadius;
  const int y22 = spot.y() + useRadius + 1;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y1][x], theECImage.saturated[y1][x], luminanceRef, saturationRef))
      count++;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y12][x], theECImage.saturated[y12][x], luminanceRef, saturationRef))
      count++;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y2][x], theECImage.saturated[y2][x], luminanceRef, saturationRef))
      count++;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y22][x], theECImage.saturated[y22][x], luminanceRef, saturationRef))
      count++;

  const int lastY = spot.y() + useRadius - 1;

  const int x1 = spot.x() - useRadius;
  const int x12 = spot.x() - useRadius;
  const int x2 = spot.x() + useRadius;
  const int x22 = spot.x() + useRadius;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y][x1], theECImage.saturated[y][x1], luminanceRef, saturationRef))
      count++;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y][x12], theECImage.saturated[y][x12], luminanceRef, saturationRef))
      count++;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y][x2], theECImage.saturated[y][x2], luminanceRef, saturationRef))
      count++;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theRelativeFieldColors.isFieldNearWhite(theECImage.grayscaled[y][x22], theECImage.saturated[y][x22], luminanceRef, saturationRef))
      count++;

  const int allPixel = 8 * (2 * useRadius - 1);
  const float percentGreen = static_cast<float>(count) / static_cast<int>(allPixel);
  DRAW_TEXT("module:BallSpotsProvider:scanLines", spot.x(), spot.y(), 6, ColorRGBA::violet, "greenPercent: " << percentGreen);
  return percentGreen > greenPercent;
}

bool BallSpotsProvider::isSpotClearlyInsideARobot(const Vector2i& spot, const float estimatedRadius) const
{
  for(const ObstaclesImagePercept::Obstacle& obstacle : theObstaclesImagePercept.obstacles)
    if(spot.x() > obstacle.left && spot.x() < obstacle.right
       && spot.y() < obstacle.bottom - estimatedRadius - IISC::getImageLineDiameterByLowestPoint(spot.cast<float>(), theCameraInfo, theCameraMatrix, theFieldDimensions))
      return true;

  return false;
}

float BallSpotsProvider::getNeededLengthFor(const int x, const int y, Geometry::Circle& circle) const
{
  const Vector2f startPoint = Vector2i(x, y).cast<float>();
  if(!IISC::calcPossibleVisibleBallByLowestPoint(startPoint, circle, theCameraInfo, theCameraMatrix, theBallSpecification, greenEdge)
    )//|| circle.radius < minRadiusOfWantedRegion)
    return 0;
  else
    return circle.radius + (startPoint - circle.center).norm() * ballSpotDistUsage;
}

MAKE_MODULE(BallSpotsProvider, perception);
