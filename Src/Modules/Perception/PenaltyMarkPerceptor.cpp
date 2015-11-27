
#include "PenaltyMarkPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Perception/BallPercept.h"
#include <algorithm>
#include <vector>

void PenaltyMarkPerceptor::update(PenaltyMarkPercept& penaltyPoint)
{

  //scale parameters according to resolution
  const float scale = theCameraInfo.width / 320.0f;
  scaledToleratedSizeDeviation = int(toleratedSizeDeviation * scale);
  scaledWhiteSpacing = int(whiteSpacing * scale);

  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor:penaltyPointScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyMarkPerceptor:noiseRect", "drawingOnImage");
  //searchScanLines(penaltyPoint);
  searchPotentialLineSpots(penaltyPoint);
  
}

void PenaltyMarkPerceptor::searchPotentialLineSpots(PenaltyMarkPercept& penaltyPoint)
{
  PenaltyMarkPercept pp;
  std::vector<Vector2i> spotsInLines;
  for(const LineSpots::Line& line : theLineSpots.lines)
  {
    spotsInLines.insert(spotsInLines.end(), line.spotsInImg.begin(), line.spotsInImg.end());
  }

  for(const PotentialScanlineSpot& spot : thePotentialLineSpots.spots)
  {
    //todo use set with lexicographic ordering of elements
    if(std::find(spotsInLines.begin(), spotsInLines.end(), spot.spotInImg) == spotsInLines.end())
    {
      pp.position = spot.spotInImg;
      pp.positionOnField = spot.spotInField;
      pp.timeLastSeen = 0;//invalidate
      if(isPenaltyMark(pp) && isFarAwayFromFieldBoundary(pp) &&
         !isTooFarAway(pp) && !isInsideBall(pp))
      {
        pp.timeLastSeen = theFrameInfo.time;
        penaltyPoint = pp;
        return;
      }
    }
  }
  DRAWTEXT("module:PenaltyMarkPerceptor:penaltyPointScanLines", 50, 50, 7, ColorRGBA::black, "not found");
}

bool PenaltyMarkPerceptor::isFarAwayFromFieldBoundary(PenaltyMarkPercept& pp) const
{
  const int boundaryY = theFieldBoundary.getBoundaryY(pp.position.x());
  Vector2f boundaryInField;
  if(!Transformation::imageToRobot(pp.position.x(), boundaryY, theCameraMatrix, theCameraInfo, boundaryInField))
  {
    return false;
  }
  const float distance = (pp.positionOnField - boundaryInField).norm();
  if(distance < minDistToFieldBoundary)
  {
    return false;
  }
  
  return true;
}

bool PenaltyMarkPerceptor::isTooFarAway(PenaltyMarkPercept& pp) const
{
  return pp.positionOnField.norm() > maxDetectionDistance;
}

bool PenaltyMarkPerceptor::isPenaltyMark(PenaltyMarkPercept& pp)
{
  const int height = theImage.height - 3;
  int horizon = std::max(2, static_cast<int>(theImageCoordinateSystem.origin.y()));
  horizon = std::min(horizon, height);
  Vector2f relativePosition;
  if(!Transformation::imageToRobot(pp.position.x(), pp.position.y(), theCameraMatrix, theCameraInfo, relativePosition))
    return false;
  int maxWidth = static_cast<int>(Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.penaltyMarkSize, relativePosition.norm()));
  //s DRAWTEXT("module:PenaltyMarkPerceptor:penaltyPointScanLines", 60, 60, 7, ColorRGBA::black, std::to_string(maxWidth));
  const int leftLimit = 2;
  const int whiteSkipping = 5;
  const int rightLimit = theImage.width - 3;
  int skipped = 0;
  int lower, upper;
  int left = 0;
  int right = 0;
  left = right = pp.position.x();
  skipped = 0;
  while(left >= leftLimit && skipped < whiteSkipping)
  {
    if(std::abs(right - (left + skipped + 1)) > maxWidth * scaledToleratedSizeDeviation)
    {
      return false;
    }
    if(theColorTable[theImage[pp.position.y()][left]].is(ColorClasses::white))
      skipped = 0;
    else
      skipped++;
    left--;
  }
  left += skipped + 1;
  skipped = 0;
  while(right <= rightLimit && skipped < whiteSkipping)
  {
    if(std::abs(left - (right - skipped)) > maxWidth * scaledToleratedSizeDeviation)
    {
      return false;
    }
    if(theColorTable[theImage[pp.position.y()][right]].is(ColorClasses::white))
      skipped = 0;
    else
      skipped++;
    right++;
  }
  right -= skipped;
  pp.position.x() = (left + right) / 2;

  // find upper/lower => middle vertical
  lower = upper = pp.position.y();
  skipped = 0;
  while(lower <= height && skipped < whiteSkipping)
  {
    if(std::abs(upper - (lower - skipped)) > maxWidth * scaledToleratedSizeDeviation)
    {
      return false;
    }
    if(theColorTable[theImage[lower][pp.position.x()]].is(ColorClasses::white))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    lower++;
  }
  lower -= skipped;

  skipped = 0;
  while(upper >= horizon && skipped < whiteSkipping)
  {
    if(std::abs(lower - (upper + skipped + 1)) > maxWidth * scaledToleratedSizeDeviation)
    {
      return false;
    }
    if(theColorTable[theImage[upper][pp.position.x()]].is(ColorClasses::white))
      skipped = 0;
    else
      skipped++;
    upper--;
  }
  upper += skipped + 1;
  pp.position.y() = (lower + upper) / 2;
  LINE("module:PenaltyMarkPerceptor:penaltyPointScanLines",
       pp.position.x(), lower,
       pp.position.x(), upper,
       1, Drawings::solidPen, ColorRGBA::blue);
  // find left/right => middle horizontal
  LINE("module:PenaltyMarkPerceptor:penaltyPointScanLines",
       left, pp.position.y(),
       right, pp.position.y(),
       1, Drawings::solidPen, ColorRGBA::blue);
  const int minBallSpotRadius = maxWidth / scaledToleratedSizeDeviation;
  bool minRadiusReached = (right - left) >= minBallSpotRadius &&
                          (lower - upper) >= minBallSpotRadius &&
                          (right - left) >= minNumPixelVertical &&
                          (lower - upper) >= minNumPixelHorizontal;
  if(!minRadiusReached)
  {
    DRAWTEXT("module:PenaltyMarkPerceptor:penaltyPointScanLines", pp.position.x(), pp.position.y(), 7, ColorRGBA::black, "minRadius");
  }
  else
  {
    //check if region is sourounded by green which is always the case for the
    //ball spot and not for line segments
    //go with a rectangle around the region
    int yUpper = upper - scaledWhiteSpacing;
    int xRight = right + scaledWhiteSpacing;
    int yLower = lower + scaledWhiteSpacing;
    int xLeft = left - scaledWhiteSpacing;

    if(yUpper <= 2 || yLower <= 2 || xRight <= 2 || xLeft <= 2 || yUpper >= height - 2 || yLower >= height - 2 || xRight >= rightLimit - 2 || xLeft >= rightLimit - 2)
      return false;
    
    std::vector<unsigned char> yValues;
    for(int i = xLeft; i <= xRight; i++)
    {
      if(!checkIsNoise(i, yUpper, yValues))
      {
        return false;
      }
      if(!checkIsNoise(i, yLower, yValues))
      {
        return false;
      }

    }
    for(int i = yUpper; i <= yLower; i++)
    {
      if(!checkIsNoise(xRight, i, yValues))
      {
        return false;
      }
      if(!checkIsNoise(xLeft, i, yValues))
      {
        return false;
      }
    }
    RECTANGLE("module:PenaltyMarkPerceptor:penaltyPointScanLines",
              xLeft, yUpper,
              xRight, yLower,
              2, Drawings::solidPen, ColorRGBA::red);
    DRAWTEXT("module:PenaltyMarkPerceptor:penaltyPointScanLines", pp.position.x(), pp.position.y(), 7, ColorRGBA::black, std::to_string(minBallSpotRadius) + ", " + std::to_string(right - left) + ", " + std::to_string(lower - upper));
    
    const float variance = calcVariance(yValues);
    DRAWTEXT("module:PenaltyMarkPerceptor:penaltyPointScanLines", pp.position.x(), pp.position.y() + 20, 7, ColorRGBA::black, "Variance: " << variance);
    if(variance >= maxVariance)
    {
      ANNOTATION("PenaltyMarkPerceptor", "Ignored Spot (variance: " << variance);
    }
    return variance < maxVariance;
  }
  return false;
}

bool PenaltyMarkPerceptor::checkIsNoise(unsigned int x, unsigned int y, std::vector<unsigned char>& yValues)
{
  RECTANGLE("module:PenaltyMarkPerceptor:noiseRect",
            x - noiseAreaSize, y - noiseAreaSize,
            x + noiseAreaSize, y + noiseAreaSize,
            1, Drawings::solidPen, ColorRGBA::yellow);
  unsigned int numNotGreenInSurrounding = 0;
  for(int dx = -noiseAreaSize; dx <= noiseAreaSize; dx++)
  {
    for(int dy = -noiseAreaSize; dy <= noiseAreaSize; dy++)
    {
      unsigned int xpos = x + dx;
      unsigned int ypos = y + dy;
      const Image::Pixel& pixel = theImage[ypos][xpos];
      yValues.push_back(pixel.y);
      if(!theColorTable[pixel].is(ColorClasses::green))
      {
        numNotGreenInSurrounding++;
        DOT("module:PenaltyMarkPerceptor:penaltyPointScanLines", xpos, ypos, ColorRGBA::yellow, ColorRGBA::yellow);
      }
    }
  }
  if(numNotGreenInSurrounding > numNotGreenMax)
  {
    DRAWTEXT("module:PenaltyMarkPerceptor:penaltyPointScanLines", x, y, 7, ColorRGBA::black, "nonoise " + std::to_string(numNotGreenInSurrounding));
  }
  return numNotGreenInSurrounding <= numNotGreenMax;
}

float PenaltyMarkPerceptor::calcVariance(const std::vector<unsigned char>& data) const
{
  //FIXME copy&paste from LineSpotProvider
  //see http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  int n = 0;
  float Sum = 0;
  float Sum_sqr = 0;

  for(const int x : data)
  {
    ++n;
    Sum += x;
    Sum_sqr += x * x;
  }
  return (Sum_sqr - (Sum * Sum) / n) / (data.size());
}

bool PenaltyMarkPerceptor::isInsideBall(PenaltyMarkPercept & pp)
{
  if(theBallPercept.status == BallPercept::Status::seen)
  {
    return (theBallPercept.positionInImage - pp.position.cast<float>()).norm() <= theBallPercept.radiusInImage;
  } else
  {
    return false;
  }
}

MAKE_MODULE(PenaltyMarkPerceptor, perception);
