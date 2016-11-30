
#include "PenaltyMarkPerceptor3000.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(PenaltyMarkPerceptor3000, perception);

PenaltyMarkPerceptor3000::PenaltyMarkPerceptor3000()
{
  if(SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    greenPercent = 0.99f;
  }
}

void PenaltyMarkPerceptor3000::update(PenaltyMarkPercept& penaltyMark)
{
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor3000:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor3000:scanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor3000:blackCheckArea", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor3000:greenCheckArea", "drawingOnImage");

  centerSpots.clear();
  searchScanLines();
  checkAndSelect(penaltyMark);
}

void PenaltyMarkPerceptor3000::searchScanLines()
{
  for(unsigned scanLineIndex = theColorScanlineRegionsVerticalClipped.lowResStart; scanLineIndex < theColorScanlineRegionsVerticalClipped.scanlines.size(); scanLineIndex+= theColorScanlineRegionsVerticalClipped.lowResStep)
  {
    int lowestYOfCurrendArea = 0;
    int currentLengthNeeded = 0;
    for(const ScanlineRegion& region : theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].regions)
    {
      if(region.is(FieldColors::white))
      {
        if(currentLengthNeeded == 0)
        {
          lowestYOfCurrendArea = region.range.lower;
          currentLengthNeeded = static_cast<int>(IISC::getImagePenaltyMarkDiameterByLowestPoint(
            Vector2f(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, region.range.lower),
            theCameraInfo, theCameraMatrix, theFieldDimensions));

          if(!currentLengthNeeded)
            break;
        }
      }
      else
      {
        if(currentLengthNeeded != 0)
        {
          const int messuredLength = lowestYOfCurrendArea - region.range.lower;
          if(std::abs(static_cast<float>(messuredLength) / static_cast<float>(currentLengthNeeded) - 1.f) < allowedLengthDisplacement)
          {
            centerSpots.emplace_back(theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, lowestYOfCurrendArea - messuredLength / 2);
            CROSS("module:PenaltyMarkPerceptor3000:spots", theColorScanlineRegionsVerticalClipped.scanlines[scanLineIndex].x, lowestYOfCurrendArea - messuredLength / 2, 5, 5, Drawings::solidPen, ColorRGBA::yellow);
          }
        }
        lowestYOfCurrendArea = 0;
        currentLengthNeeded = 0;
      }
    }
  }
}

void PenaltyMarkPerceptor3000::checkAndSelect(PenaltyMarkPercept& penaltyMark)
{
  for(std::vector<Vector2i>::reverse_iterator ptr = centerSpots.rbegin(); ptr != centerSpots.rend(); ptr++)
  {
    float heightRadius, lengthRadius;
    if(!IISC::calculateImagePenaltyMeasurementsByCenter(ptr->cast<float>(), lengthRadius, heightRadius, theCameraInfo, theCameraMatrix, theFieldDimensions)
       || isDuplicative(ptr, lengthRadius)
       || isInBall(*ptr)
       || !correctWithScanSidewards(*ptr, static_cast<int>(lengthRadius / 2))
       || !checkBlackInside(*ptr, static_cast<int>(heightRadius), static_cast<int>(lengthRadius))
       || !checkGreenAround(*ptr, static_cast<int>(heightRadius), static_cast<int>(lengthRadius)))
      continue;

    return fillPenaltyMark(penaltyMark, *ptr);
  }
}

void PenaltyMarkPerceptor3000::fillPenaltyMark(PenaltyMarkPercept& penaltyMark, const Vector2i& spot) const
{
  penaltyMark.position = spot;
  if(Transformation::imageToRobot(spot, theCameraMatrix, theCameraInfo, penaltyMark.positionOnField))
    penaltyMark.timeLastSeen = theFrameInfo.time;
}

bool PenaltyMarkPerceptor3000::correctWithScanSidewards(Vector2i& initialPoint, const int radiusLength) const
{
  const int maxScanLength = static_cast<int>(radiusLength * scanLengthRadiusFactor);

  int leftMaximum(0), rightMaximum(theECImage.colored.width);
  theBodyContour.clipLeft(leftMaximum, initialPoint.y());
  theBodyContour.clipRight(rightMaximum, initialPoint.y());

  const int maxLeftScanLength = std::min(maxScanLength, initialPoint.x() - leftMaximum);
  const int maxRightScanLength = std::min(maxScanLength, rightMaximum - initialPoint.x());

  unsigned foundGoodPixel = 0;
  int leftScanLength = 0;
  scanSpotOneDirection(initialPoint, leftScanLength, maxLeftScanLength, foundGoodPixel,
  [](const Vector2i & spot, const int currentLength) {return int(spot.x() - currentLength); },
  [](const Vector2i & spot, const int currentLength) {return int(spot.y()); });

  int rightScanLength = 0;
  scanSpotOneDirection(initialPoint, rightScanLength, maxRightScanLength, foundGoodPixel,
  [](const Vector2i & spot, const int currentLength) {return int(spot.x() + currentLength); },
  [](const Vector2i & spot, const int currentLength) {return int(spot.y()); });

  initialPoint.x() -= (leftScanLength - rightScanLength) / 2;

  const float displacement = std::abs(static_cast<float>(leftScanLength + rightScanLength) / static_cast<float>(2 * radiusLength) - 1.f);
  CROSS("module:PenaltyMarkPerceptor3000:spots", initialPoint.x(), initialPoint.y(), 5, 5, Drawings::solidPen, ColorRGBA::red);
  DRAWTEXT("module:PenaltyMarkPerceptor3000:spots", initialPoint.x() + 10, initialPoint.y(), 7, ColorRGBA::red, "displacement: " << displacement);

  return  displacement < allowedLengthDisplacement;
}

void PenaltyMarkPerceptor3000::scanSpotOneDirection(const Vector2i& spot, int& currentLength, const int& maxLength,
    unsigned& goodPixelCounter,
    int(*getX)(const Vector2i& spot, const int currentLength),
    int(*getY)(const Vector2i& spot, const int currentLength)) const
{
  unsigned currentSkipped = 0;
  while(checkPixel(theECImage.colored[getY(spot, currentLength)][getX(spot, currentLength)], goodPixelCounter, currentSkipped)
        && ++currentLength <= maxLength);
  currentLength -= currentSkipped;

  LINE("module:PenaltyMarkPerceptor3000:scanLines", spot.x(), spot.y(), getX(spot, maxLength), getY(spot, maxLength),
       1, Drawings::solidPen, ColorRGBA::yellow);
  LINE("module:PenaltyMarkPerceptor3000:scanLines", spot.x(), spot.y(), getX(spot, currentLength), getY(spot, currentLength),
       1, Drawings::solidPen, ColorRGBA::red);
}

bool PenaltyMarkPerceptor3000::checkPixel(const PixelTypes::ColoredPixel& pixel, unsigned& goodPixelCounter, unsigned& currentSkipped) const
{
  if(pixel == FieldColors::white)
  {
    currentSkipped = 0;
    ++goodPixelCounter;
  }
  else
    ++currentSkipped;

  return currentSkipped < maxNumberOfSkippablePixel;
}

bool PenaltyMarkPerceptor3000::isDuplicative(std::vector<Vector2i>::reverse_iterator ptr, const float estimatedRadius) const
{
  for(std::vector<Vector2i>::reverse_iterator otherPtr = ptr + 1; otherPtr != centerSpots.rend(); otherPtr++)
    if((*ptr - *otherPtr).squaredNorm() < sqr(estimatedRadius) * 2.f)
      return true;

  return false;
}

bool PenaltyMarkPerceptor3000::checkGreenAround(const Vector2i& spot, const int radiusHeight, const int radiusLength) const
{
  int useHeightRadius = additionalRadiusForGreenCheck + radiusHeight;
  int useLenghtRadius = additionalRadiusForGreenCheck + radiusLength;
  if(useLenghtRadius >= spot.x() - 1 || useHeightRadius >= spot.y() - 1 ||
     spot.x() + 1 + useLenghtRadius >= theECImage.colored.width ||
     spot.y() + 1 + useHeightRadius >= theECImage.colored.height)
    return false;

  int count(0);
  const int lastX = spot.x() + useLenghtRadius - 1;

  const int y1 = spot.y() - useHeightRadius;
  const int y12 = spot.y() - useHeightRadius - 1;
  const int y2 = spot.y() + useHeightRadius;
  const int y22 = spot.y() + useHeightRadius + 1;

  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", spot.x() - useLenghtRadius + 1, y1, lastX, y1,1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", spot.x() - useLenghtRadius + 1, y12, lastX, y12, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", spot.x() - useLenghtRadius + 1, y2, lastX, y2, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", spot.x() - useLenghtRadius + 1, y22, lastX, y22, 1, Drawings::solidPen, ColorRGBA::gray);
  
  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y1][x] == FieldColors::green)
      count++;

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y12][x] == FieldColors::green)
      count++;

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y2][x] == FieldColors::green)
      count++;

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(theECImage.colored[y22][x] == FieldColors::green)
      count++;

  const int lastY = spot.y() + useHeightRadius - 1;

  const int x1 = spot.x() - useLenghtRadius;
  const int x12 = spot.x() - useLenghtRadius - 1;
  const int x2 = spot.x() + useLenghtRadius;
  const int x22 = spot.x() + useLenghtRadius + 1;

  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", x1, spot.y() - useHeightRadius + 1, x1, lastY, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", x12, spot.y() - useHeightRadius + 1, x12, lastY, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", x2, spot.y() - useHeightRadius + 1, x2, lastY, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:greenCheckArea", x22, spot.y() - useHeightRadius + 1, x22, lastY, 1, Drawings::solidPen, ColorRGBA::gray);

  for(int y = spot.y() - useHeightRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x1] == FieldColors::green)
      count++;

  for(int y = spot.y() - useHeightRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x12] == FieldColors::green)
      count++;

  for(int y = spot.y() - useHeightRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x2] == FieldColors::green)
      count++;

  for(int y = spot.y() - useHeightRadius + 1; y <= lastY; y++)
    if(theECImage.colored[y][x22] == FieldColors::green)
      count++;

  const int allPixel = 4 * (2 * useLenghtRadius - 1 + 2 * useHeightRadius - 1);
  const float percentGreen = static_cast<float>(count) / static_cast<int>(allPixel);

  DRAWTEXT("module:PenaltyMarkPerceptor3000:spots", spot.x() + 10, spot.y() + 8, 7, ColorRGBA::red, "percentGreen: " << percentGreen);

  return percentGreen > greenPercent;
}

bool PenaltyMarkPerceptor3000::checkBlackInside(const Vector2i& spot, const int radiusHeight, const int radiusLength) const
{
  int useHeightRadius = radiusHeight / 2;
  int useLenghtRadius = radiusLength / 2;
  if(useLenghtRadius >= spot.x() - 1 || useHeightRadius >= spot.y() - 1 ||
     spot.x() + 1 + useLenghtRadius >= theECImage.colored.width ||
     spot.y() + 1 + useHeightRadius >= theECImage.colored.height)
    return false;

  unsigned count(0);
  const int lastX = spot.x() + useLenghtRadius - 1;

  const int y1 = spot.y() - useHeightRadius;
  const int y12 = spot.y() - static_cast<int>(0.3f * useHeightRadius);
  const int y2 = spot.y() + useHeightRadius;
  const int y22 = spot.y() + static_cast<int>(0.3f * useHeightRadius);

  LINE("module:PenaltyMarkPerceptor3000:blackCheckArea", spot.x() - useLenghtRadius + 1, y1, lastX, y1, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:blackCheckArea", spot.x() - useLenghtRadius + 1, y12, lastX, y12, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:blackCheckArea", spot.x() - useLenghtRadius + 1, y2, lastX, y2, 1, Drawings::solidPen, ColorRGBA::gray);
  LINE("module:PenaltyMarkPerceptor3000:blackCheckArea", spot.x() - useLenghtRadius + 1, y22, lastX, y22, 1, Drawings::solidPen, ColorRGBA::gray);

  static const char definitlyNotBlack = 0x1;
  ASSERT(FieldColors::green & definitlyNotBlack && FieldColors::white & definitlyNotBlack
    && !(FieldColors::black & definitlyNotBlack) && !(FieldColors::none & definitlyNotBlack));

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(!(theECImage.colored[y1][x] & definitlyNotBlack))
      count++;

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(!(theECImage.colored[y12][x] & definitlyNotBlack))
      count++;

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(!(theECImage.colored[y2][x] & definitlyNotBlack))
      count++;

  for(int x = spot.x() - useLenghtRadius + 1; x <= lastX; x++)
    if(!(theECImage.colored[y22][x] & definitlyNotBlack))
      count++;
  
  DRAWTEXT("module:PenaltyMarkPerceptor3000:spots", spot.x() + 10, spot.y() - 8, 7, ColorRGBA::red, "blackCount: " << count);

  return count <= maxAllowedBlackPixel;
}

bool PenaltyMarkPerceptor3000::isInBall(const Vector2i& spot) const
{
  Vector2f pointInImage;
  if(Transformation::robotToImage(theBallPrediction.position, theCameraMatrix, theCameraInfo, pointInImage))
  {
    const float radius = IISC::getImageBallRadiusByCenter(pointInImage, theCameraInfo, theCameraMatrix, theFieldDimensions);
    return (pointInImage - spot.cast<float>()).squaredNorm() < sqr(radius * ballRadiusMultiplier);
  }
  return false;
}
