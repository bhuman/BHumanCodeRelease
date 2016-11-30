/**
 * @file BallSpotProvider17.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
#include "BallSpotProvider17.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include <algorithm>

void BallSpotProvider17::update(BallSpots& ballSpots)
{
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider17:scanLines", "drawingOnImage");

  ballSpots.ballSpots.clear();
  searchScanLines(ballSpots);
}

void BallSpotProvider17::searchScanLines(BallSpots& ballSpots) const
{
  //todo bodyy and fieldline
  const unsigned step = theColorScanlineRegionsVerticalClipped.lowResStep > 1 ? theColorScanlineRegionsVerticalClipped.lowResStep / 2 : 1;
  const unsigned start = theColorScanlineRegionsVerticalClipped.lowResStart >= step ? theColorScanlineRegionsVerticalClipped.lowResStart - step : theColorScanlineRegionsVerticalClipped.lowResStart;
  Geometry::Circle circle;

  for(unsigned scanLineIndex = start; scanLineIndex < theColorScanlineRegionsVerticalClipped.scanlines.size(); scanLineIndex += step)
  {
    int lowestYOfCurrendArea = 0;
    int currentLengthNeeded = 0;
    for(const ScanlineRegion& region : theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].regions)
    {
      if(!region.is(FieldColors::green))
      {
        if(currentLengthNeeded == 0)
        {
          lowestYOfCurrendArea = region.range.lower;
          currentLengthNeeded = static_cast<int>(getNeededLengthFor(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, region.range.lower, circle));
          if(!currentLengthNeeded)
            break;
        }
      }
      else
      {
        if(currentLengthNeeded != 0)
        {
          Vector2i spot;
          bool foundSpot = true;

          if(lowestYOfCurrendArea - region.range.lower > currentLengthNeeded)
            ballSpots.ballSpots.emplace_back(circle.center.cast<int>());
          else if(currentLengthNeeded != 0 && lowestYOfCurrendArea == theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].regions.front().range.lower &&
            (scanLineIndex - start) % step == 0 && lowestYOfCurrendArea - region.range.lower > currentLengthNeeded / 2)
            ballSpots.ballSpots.emplace_back(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, (lowestYOfCurrendArea + region.range.lower) / 2);
          else
            foundSpot = false;

          if(foundSpot)
          {
            bool deleteSpot;

            deleteSpot = isLastDuplicative(ballSpots, static_cast<int>(circle.radius * minAllowedDistanzRadiusRelation));

            if(deleteSpot || isSpotClearlyInsideARobot(ballSpots.ballSpots.back(), circle.radius))
              goto erase;

            deleteSpot = !correctWithScanLeftAndRight(ballSpots.ballSpots.back(), circle);

            if(deleteSpot)
              goto erase;

            deleteSpot = isLastDuplicative(ballSpots, static_cast<int>(circle.radius * minAllowedDistanzRadiusRelation));

            if(deleteSpot)
              goto erase;

            if(currentLengthNeeded < minRadiusOfWantedRegion)
              deleteSpot = !checkGreenAround(ballSpots.ballSpots.back(), circle.radius);

          erase:
            if(deleteSpot || isSpotClearlyInsideARobot(ballSpots.ballSpots.back(), circle.radius))
              ballSpots.ballSpots.erase(ballSpots.ballSpots.end() - 1);
          }
        }
        // if(theCameraInfo.camera == CameraInfo::lower && )

        //TODO if head reached
        lowestYOfCurrendArea = 0;
        currentLengthNeeded = 0;
      }
    }
  }
}

bool BallSpotProvider17::correctWithScanLeftAndRight(Vector2i& initialPoint, const Geometry::Circle& circle) const
{
  const int maxScanLength = static_cast<int>(circle.radius * scanLengthRadiusFactor);

  int leftMaximum(0), rightMaximum(theECImage.colored.width);
  theBodyContour.clipLeft(leftMaximum, initialPoint.y());
  theBodyContour.clipRight(rightMaximum, initialPoint.y());

  const int maxLeftScanLength = std::min(maxScanLength, initialPoint.x() - leftMaximum);
  const int maxRightScanLength = std::min(maxScanLength, rightMaximum - initialPoint.x());

  unsigned foundGoodPixel = 0;
  unsigned foundNeutralPixel = 0;
  int leftScanLength = 0;
  scanBallSpotOneDirection(initialPoint, leftScanLength, maxLeftScanLength, foundGoodPixel, foundNeutralPixel,
  [](const Vector2i & spot, const int currentLength) {return int(spot.x() - currentLength); },
  [](const Vector2i & spot, const int currentLength) {return int(spot.y()); });

  int rightScanLength = 0;
  scanBallSpotOneDirection(initialPoint, rightScanLength, maxRightScanLength, foundGoodPixel, foundNeutralPixel,
  [](const Vector2i & spot, const int currentLength) {return int(spot.x() + currentLength); },
  [](const Vector2i & spot, const int currentLength) {return int(spot.y()); });

  initialPoint.x() -= (leftScanLength - rightScanLength) / 2;
  const float noice = 1.f - static_cast<float>(foundGoodPixel) / static_cast<float>(leftScanLength + rightScanLength - foundNeutralPixel);
  const float goodNeutralRatio = static_cast<float>(foundGoodPixel) / static_cast<float>(foundNeutralPixel + foundGoodPixel);
  const float foundDiameterPercentage = static_cast<float>(leftScanLength + rightScanLength) / static_cast<float>(2 * circle.radius);

  DRAWTEXT("module:BallSpotProvider17:scanLines", initialPoint.x() + 10, initialPoint.y() - 4, 6, ColorRGBA::red, "noice:" << noice);
  DRAWTEXT("module:BallSpotProvider17:scanLines", initialPoint.x() + 10, initialPoint.y() + 5, 6, ColorRGBA::red, "foundDiameterPercentage:" << foundDiameterPercentage);
  DRAWTEXT("module:BallSpotProvider17:scanLines", initialPoint.x() + 10, initialPoint.y() + 9, 6, ColorRGBA::red, "goodNeutralRatio:" << goodNeutralRatio);

  return noice < noiceThreshold && foundDiameterPercentage > minFoundDiameterPercentage && goodNeutralRatio > minGoodNeutralRatio;
}

void BallSpotProvider17::scanBallSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
  unsigned& goodPixelCounter, unsigned& neutralPixelCounter,
  int(*getX) (const Vector2i& spot, const int currentLength),
  int(*getY) (const Vector2i& spot, const int currentLength)) const
{
  unsigned currentSkipped = 0;
  unsigned currentSkippedGreen = 0;
  while(checkPixel(theECImage.colored[getY(spot, currentLength)][getX(spot, currentLength)], goodPixelCounter, neutralPixelCounter, currentSkippedGreen, currentSkipped)
        && ++currentLength <= maxLength);
  currentLength -= currentSkipped;

  LINE("module:BallSpotProvider17:scanLines", spot.x(), spot.y(), getX(spot, maxLength), getY(spot, maxLength),
       1, Drawings::solidPen, ColorRGBA::yellow);
  LINE("module:BallSpotProvider17:scanLines", spot.x(), spot.y(), getX(spot, currentLength), getY(spot, currentLength),
       1, Drawings::solidPen, ColorRGBA::red);
}

bool BallSpotProvider17::checkPixel(const PixelTypes::ColoredPixel& pixel, unsigned& goodPixelCounter, unsigned& neutralPixelCounter, unsigned& currentSkippedGreen, unsigned& currentSkipped) const
{
  if(pixel == FieldColors::white)
  {
    currentSkipped = 0;
    currentSkippedGreen = 0;
    ++goodPixelCounter;
  }
  else if(pixel == FieldColors::black)
  {
    if(!blackPixelsAreNeutral)
    {
      currentSkipped = 0;
      currentSkippedGreen = 0;
      ++goodPixelCounter;
    }
    else
      ++neutralPixelCounter;
  }
  else if(pixel == FieldColors::green)
  {
    ++currentSkipped;
    ++currentSkippedGreen;
  }
  else if(!allowColorNon)
    ++currentSkipped;
  else
    ++neutralPixelCounter;

  return currentSkipped < maxNumberOfSkippablePixel && currentSkippedGreen < maxNumberOfSkippableGreenPixel;
}

bool BallSpotProvider17::isLastDuplicative(const BallSpots& ballSpots, const int minAllowedDistanz) const
{
  if(ballSpots.ballSpots.size() < 2)
    return false;

  const int sqaredAllowdDistanz = sqr(minAllowedDistanz);
  const Vector2i& spotToCheck = ballSpots.ballSpots.back();
  for(auto ptr = ballSpots.ballSpots.begin(); ptr < ballSpots.ballSpots.end() - 1; ptr++)
    if((*ptr - spotToCheck).squaredNorm() < sqaredAllowdDistanz)
      return true;

  return false;
}

bool BallSpotProvider17::checkGreenAround(const Vector2i& spot, const float radius) const
{
  int useRadius = additionalRadiusForGreenCheck + static_cast<int>(radius);
  if(useRadius >= spot.x() - 1 || useRadius >= spot.y() - 1 ||
     spot.x() + 1 + useRadius >= theECImage.colored.width ||
     spot.y() + 1 + useRadius >= theECImage.colored.height)
    return false;

  int count(0);
  const int lastX = spot.x() + useRadius - 1;

  const int y1 = spot.y() - useRadius;
  const int y12 = spot.y() - useRadius - 1;
  const int y2 = spot.y() + useRadius;
  const int y22 = spot.y() + useRadius + 1;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y1][x] == FieldColors::green)
      count++;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y12][x] == FieldColors::green)
      count++;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y2][x] == FieldColors::green)
      count++;

  for(int x = spot.x() - useRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y22][x] == FieldColors::green)
      count++;

  const int lastY = spot.y() + useRadius - 1;

  const int x1 = spot.x() - useRadius;
  const int x12 = spot.x() - useRadius;
  const int x2 = spot.x() + useRadius;
  const int x22 = spot.x() + useRadius;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x1] == FieldColors::green)
      count++;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x12] == FieldColors::green)
      count++;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x2] == FieldColors::green)
      count++;

  for(int y = spot.y() - useRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x22] == FieldColors::green)
      count++;

  const int allPixel = 8 * (2 * useRadius - 1);
  const float percentGreen = static_cast<float>(count) / static_cast<int>(allPixel);

  return percentGreen > greenPercent;
}


bool BallSpotProvider17::isSpotClearlyInsideARobot(const Vector2i& spot, const float estimatedRadius) const
{
  for(const PlayersPercept::Player& player : thePlayersPercept.players)
    if(spot.x() > player.x1FeetOnly && spot.x() < player.x2FeetOnly)
      if(spot.y() < player.y2 - estimatedRadius - IISC::getImageLineDiameterByLowestPoint(spot.cast<float>(), theCameraInfo, theCameraMatrix, theFieldDimensions))
        return true;

  return false;
}


float BallSpotProvider17::getNeededLengthFor(const int x, const int y, Geometry::Circle& circle) const
{
  const Vector2f startPoint = Vector2i(x, y).cast<float>();
  if(!IISC::calcPossibleVisibleBallByLowestPoint(startPoint, circle, theCameraInfo, theCameraMatrix, theFieldDimensions, greenEdge)
    )//|| circle.radius < minRadiusOfWantedRegion)
    return 0;
  else
    return circle.radius + (startPoint - circle.center).norm() * ballSpotDistUsage;
}

MAKE_MODULE(BallSpotProvider17, perception)
