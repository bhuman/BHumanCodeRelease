/**
 * @file PlayersPerceptor.cpp
 * @ author Michel Bartsch, Vitali Gutsch
 */

#include "PlayersPerceptor.h"
#include "Representations/Configuration/FieldColors.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include <algorithm>

void PlayersPerceptor::update(PlayersPercept& playersPercept)
{
  DECLARE_DEBUG_DRAWING("module:PlayersPerceptor", "drawingOnImage");

  playersPercept.players.clear();

  if(!theCameraMatrix.isValid || !calcRealWorldSizes())
    return;

  ASSERT(theCameraInfo.width == theECImage.grayscaled.width);
  for(int i = 0, x = 0, xEnd = theCameraInfo.width - xStep; x < xEnd; i++, x += xStep)
  {
    scanVerticalLine(i, x);
    if(verticalLines[i].y != 0)
    {
      CROSS("module:PlayersPerceptor", x, verticalLines[i].y, 1, 1, Drawings::solidPen, ColorRGBA::orange);
    }
  }

  while(combineVerticalLines(playersPercept));

  for(auto i = playersPercept.players.begin(); i != playersPercept.players.end();)
  {
    if(theCameraInfo.camera == CameraInfo::lower)
      scanJersey(*i, 0, true);
    else if(i->fallen)
      scanJersey(*i, 0.5, true);
    else if(theOwnTeamInfo.teamColor == TEAM_BLACK || theOpponentTeamInfo.teamColor == TEAM_BLACK)
      scanJersey(*i, refereeY, false) || scanJersey(*i, jerseyYFirst, true) || scanJersey(*i, jerseyYSecond, true) || scanJersey(*i, jerseyYThird, true);
    else
      scanJersey(*i, jerseyYFirst, true) || scanJersey(*i, jerseyYSecond, true) || scanJersey(*i, jerseyYThird, true);

    const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(Vector2i(i->realCenterX, i->y2));
    const Vector2f correctedLeft = theImageCoordinateSystem.toCorrected(Vector2i(i->x1FeetOnly, i->y2));
    const Vector2f correctedRight = theImageCoordinateSystem.toCorrected(Vector2i(i->x2FeetOnly, i->y2));

    if(Transformation::imageToRobot(correctedCenter, theCameraMatrix, theCameraInfo, i->centerOnField) &&
       Transformation::imageToRobot(correctedLeft, theCameraMatrix, theCameraInfo, i->leftOnField) &&
       Transformation::imageToRobot(correctedRight, theCameraMatrix, theCameraInfo, i->rightOnField))
    {
      i->centerOnField.normalize(i->centerOnField.norm() + robotDepth);
      ++i;
    }
    else
      i = playersPercept.players.erase(i);
  }
}

bool PlayersPerceptor::calcRealWorldSizes()
{
  horizon = (int)theImageCoordinateSystem.origin.y();
  Vector2f pCorrected;
  Vector2f point;
  if(horizon < 1)
  {
    if(theCameraInfo.camera == CameraInfo::lower)
    {
      farthestXGap = xGapMillisLower;
      farthestYGap = yGapMillisLower;
      farthestMinWidth = minWidthMillisLower;
    }
    else
    {
      farthestXGap = xGapMillisUpper;
      farthestYGap = yGapMillisUpper;
      farthestMinWidth = minWidthMillisUpper;
    }
    
    horizon = 1;
    pCorrected = theImageCoordinateSystem.toCorrected(Vector2i(theCameraInfo.width / 2, horizon));
    if(!Transformation::imageToRobot(pCorrected, theCameraMatrix, theCameraInfo, point))
    {
      farthestXGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestXGap, (float)minFarthestMillis);
      farthestYGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestYGap, (float)minFarthestMillis);
      farthestMinWidth = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestMinWidth, (float)minFarthestMillis);
    }
    else
    {
      farthestXGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestXGap, point.norm());
      farthestYGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestYGap, point.norm());
      farthestMinWidth = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestMinWidth, point.norm());
    }
  }
  else if(horizon >= theCameraInfo.height - 1)
  {
    return false;
  }
  else
  {
    farthestXGap = 0;
    farthestYGap = 0;
    farthestMinWidth = 0;
  }
  pCorrected = theImageCoordinateSystem.toCorrected(Vector2i(theCameraInfo.width / 2, theCameraInfo.height - 1));
  if(!Transformation::imageToRobot(pCorrected, theCameraMatrix, theCameraInfo, point))
  {
    return false;
  }
  else
  {
    nearestXGap = (theCameraInfo.camera == CameraInfo::lower ? xGapMillisLower : xGapMillisUpper);
    nearestXGap = (int) Geometry::getSizeByDistance(theCameraInfo, (float) nearestXGap, point.norm());
    nearestYGap = (theCameraInfo.camera == CameraInfo::lower ? yGapMillisLower : yGapMillisUpper);
    nearestYGap = (int) Geometry::getSizeByDistance(theCameraInfo, (float) nearestYGap, point.norm());
    nearestMinWidth = (theCameraInfo.camera == CameraInfo::lower ? minWidthMillisLower : minWidthMillisUpper);
    nearestMinWidth = (int) Geometry::getSizeByDistance(theCameraInfo, (float) nearestMinWidth, point.norm());
    if(nearestMinWidth < 0)
    {
      //camera was looking very high,
      //the lowest point was too far away and produced overflow.
      //Because of the earth curvature, this point most likely never touches the earth.
      // -> Remove this for perception of mars rovers ;)
      return false;
    }
  }

  return true;
}

void PlayersPerceptor::scanVerticalLine(int index, int x)
{
  int maxY = 0;
  int y = std::max(theFieldBoundary.getBoundaryY(x) + yOffset, 0);
  int yEnd = theCameraInfo.height - 1;
  theBodyContour.clipBottom(x, yEnd, yEnd);
  float grow = (y > 0 ? growBaseUpper : growBaseLower) * nearestMinWidth / std::max(theCameraInfo.height - y, 1);
  float step = (y > 0 ? yStepBaseUpper : yStepBaseLower);
  int shortRange = 0;
  int longRange = 0;
  bool onGround =  false;
  bool reachedBottom = false;

  for(int i = 1; true; i++)
  {
    if(y >= yEnd)
    {
      if(!reachedBottom)
      {
        reachedBottom = true;
        y = yEnd;
      }
      else
      {
        break;
      }
    }
    ASSERT(x >= 0 && x < theCameraInfo.width && y >= 0 && y < theCameraInfo.height);
    //RECTANGLE("module:PlayersPerceptor", x, y, x, y, 0, Drawings::solidPen, ColorRGBA::yellow);
    const PixelTypes::ColoredPixel& pixel = theECImage.colored[y][x];
    if(pixel == FieldColors::green) //|| theColorTable[pixel].is(FieldColors::orange))
    {
      shortRange = 0;
      longRange -= greenMinus;
      if(longRange < -downRange)
      {
        longRange = -downRange;
        onGround =  true;
      }
    }
    else
    {
      shortRange++;
      longRange++;
    }
    if(i >= minHeight && shortRange >= lastNongreen && longRange >= 0)
    {
      maxY = y;
      verticalLines[index].onGround =  onGround;
    }
    /*  Scanning only near fieldBoundary,
        much better performance but problems with fallen players
    else if(longRange < -downRange)
    {
      break;
    }*/
    step += grow;
    y += (int)step;
  }

  verticalLines[index].y = (short)maxY;
  verticalLines[index].noFeet = (maxY == yEnd);
}

bool PlayersPerceptor::combineVerticalLines(PlayersPercept& playersPercept)
{
  int maxY = 0;
  int maxYIndex;
  int xGap;
  int yGap;
  int connected;
  int minWidth;
  int gap;
  int left;
  int right;
  int width;
  int y;
  int top;

  // find lowest spot
  for(int i = 0, iEnd = (theCameraInfo.width - xStep) / xStep; i < iEnd; i++)
  {
    if(verticalLines[i].y > maxY)
    {
      maxY = verticalLines[i].y;
      maxYIndex = i;
    }
  }
  if(maxY == 0)
  {
    return false;
  }

  // find left and right dimension
  float distanceFactor = (maxY - horizon) / (float)(theCameraInfo.height - horizon);
  xGap = (int)(distanceFactor * (nearestXGap - farthestXGap) + farthestXGap);
  yGap = (int)(distanceFactor * (nearestYGap - farthestYGap) + farthestYGap);
  minWidth = (int)(distanceFactor * (nearestMinWidth - farthestMinWidth) + farthestMinWidth);
  left = maxYIndex - 1;
  gap = 0;
  connected = 0;
  for(; left >= 0; left--)
  {
    if(!verticalLines[left].y || verticalLines[left].y < maxY - yGap)
    {
      gap += xStep;
      if(gap > xGap)
      {
        break;
      }
    }
    else
    {
      if(gap == 0)
      {
        connected++;
      }
      else
      {
        gap = 0;
      }
    }
  }
  left = left * xStep + gap;
  right = maxYIndex + 1;
  gap = 0;
  for(int rightEnd = (theCameraInfo.width - 1) / xStep; right < rightEnd; right++)
  {
    if(!verticalLines[right].y || verticalLines[right].y < maxY - yGap)
    {
      gap += xStep;
      if(gap > xGap)
      {
        break;
      }
    }
    else
    {
      if(gap == 0)
      {
        connected++;
      }
      else
      {
        gap = 0;
      }
    }
  }
  right = right * xStep - gap;
  connected *= xStep;

  // check width and connected
  width = (right - 1) - left;
  if(width < minWidth || connected < minConnected * minWidth)
  {
    verticalLines[maxYIndex].y = 0;
    return true;
  }

  // add ignore space
  int realCenterX = left + (right - left) / 2;
  int feetRight = right;
  int feetLeft = left;
  left -= (int)(width * addedWidth);
  if(left < 0)
  {
    left = 0;
  }
  right += (int)(width * addedWidth);
  if(right >= theCameraInfo.width)
  {
    right = theCameraInfo.width - 1;
  }

  // find exact bottom
  y = maxY;
  gap = 0;
  int yEnd = theCameraInfo.height;
  theBodyContour.clipBottom(maxYIndex * xStep, yEnd, yEnd);
  for(int x = maxYIndex * xStep; y < yEnd; y++)
  {
    //RECTANGLE("module:PlayersPerceptor", x, y, x, y, 0, Drawings::solidPen, ColorRGBA::yellow);
    const PixelTypes::ColoredPixel& pixel = theECImage.colored[y][x];
    if(pixel == FieldColors::green)//|| theColorTable[pixel].is(FieldColors::orange))
    {
      if(++gap > 1)
      {
        break;
      }
    }
    else
    {
      gap = 0;
    }
  }
  y--;
  y -= gap;

  // guess top
  top = horizon - ((y - horizon) >> 3);
  if(top < 1)
  {
    top = 1;
  }

  // remove scanlines within obstacle and sum onGround for fallen
  int onGroundSum = 0;
  for(int i = left / xStep, iEnd = right / xStep; i <= iEnd; i++)
  {
    verticalLines[i].y = 0;
    if(verticalLines[i].onGround)
    {
      onGroundSum++;
    }
  }

  // register obstacle
  PlayersPercept::Player obstacle = PlayersPercept::Player();
  obstacle.x1 = left;
  obstacle.x2 = right;
  obstacle.y1 = top;
  obstacle.y2 = y;
  obstacle.realCenterX = realCenterX;
  obstacle.x1FeetOnly = feetLeft;
  obstacle.x2FeetOnly = feetRight;
  obstacle.lowerCamera = theCameraInfo.camera == CameraInfo::lower;
  obstacle.detectedJersey = false;
  obstacle.detectedFeet = !verticalLines[maxYIndex].noFeet;
  obstacle.ownTeam = false;
  obstacle.fallen = (onGroundSum >= ((right - left) / xStep) * onGroundMajority);
  playersPercept.players.push_back(obstacle);

  return true;
}

bool PlayersPerceptor::scanJersey(PlayersPercept::Player& obstacle, float height, bool searchForPlayer)
{
  Vector2f vanishingPoint = computeVanishingPointZ();

  // "uncorrect" vanishing point with linearized motion distortion model
  vanishingPoint = theImageCoordinateSystem.fromCorrectedLinearized(vanishingPoint);

  int y = (int)(height * (obstacle.y2 - obstacle.y1) + obstacle.y1);
  Vector2f bottom((static_cast<float>(obstacle.x1FeetOnly) + static_cast<float>(obstacle.x2FeetOnly)) / 2.f, static_cast<float>(obstacle.y2));

  Vector2f top = bottom + (vanishingPoint - bottom).normalized() * (static_cast<float>(y) - bottom.y());
  float halfWidth = static_cast<float>(obstacle.x2FeetOnly - obstacle.x1FeetOnly) / 4.f;
  float x = top.x() - halfWidth;
  float xEnd = top.x() + halfWidth;
  float step = std::max(1.f, (xEnd - x) / jerseyScanConcentration);
  int opponent = 0;
  int teammate = 0;
  int white = 0;

  bool ownJerseyIsBlack = theOwnTeamInfo.teamColor == TEAM_BLACK;
  bool opponentJerseyIsBlack = theOwnTeamInfo.teamColor == TEAM_BLACK;
  ASSERT(theOwnTeamInfo.teamColor < 4 && theOpponentTeamInfo.teamColor < 4);
  Image::Pixel ownColor = colors[theOwnTeamInfo.teamColor];
  Image::Pixel opponentColor = colors[theOpponentTeamInfo.teamColor];

  int colorNeeded = std::max(1, (int)(jerseyNeeded * jerseyScanConcentration));

  for(; x <= xEnd; x += step)
  {
    Image::Pixel current = theImage.getFullSizePixel(y, static_cast<int>(x));
    ColorModelConversions::fromYUVToHSI(current.y, current.cb, current.cr, current.h, current.s, current.i);

    RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::yellow);

    if(current.i > minWhiteI && current.s < maxWhiteS)
    {
      RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::white);
      white++;
    }
    else if(current.h > minGreenH && current.h < maxGreenH && current.i > minGreenI)
    {
      RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::green);
    }
    else if(ownJerseyIsBlack)
    {
      analyzePixel((int) x, y, current, opponentColor, ownColor.i, searchForPlayer, teammate, opponent);
    }
    else if(opponentJerseyIsBlack)
    {
      analyzePixel((int) x, y, current, ownColor, opponentColor.i, searchForPlayer, opponent, teammate);
    }
    else if(current.i > maxBlackI)
    {
      int diffToOwnH = (int) std::min(std::min(std::abs(current.h - ownColor.h), (std::abs(current.h - ownColor.h + 256))), (std::abs(current.h - ownColor.h - 256)));
      int diffToOpponentsH = (int) std::min(std::min(std::abs(current.h - opponentColor.h), (std::abs(current.h - opponentColor.h + 256))), (std::abs(current.h - opponentColor.h - 256)));
      int maxRangeH = this->maxRangeH * (current.i < maxDarkI ? 2 : 1);
      if(diffToOpponentsH < maxRangeH && diffToOwnH >= maxRangeH && current.s > maxGrayS)
      {
        RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
        opponent++;
      }
      else if(diffToOpponentsH >= maxRangeH && diffToOwnH < maxRangeH && current.s > maxGrayS)
      {
        RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::blue);
        teammate++;
      }
      else if(diffToOpponentsH < maxRangeH && diffToOwnH < maxRangeH && current.s > maxGrayS)
      {
        if(diffToOwnH < diffToOpponentsH)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::blue);
          teammate++;
        }
        else
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
          opponent++;
        }
      }
    }
  }

  // when there is more color of a jersey in the player as needed to
  // find a jersey and the difference between the two
  // jersey colors is more than half of the needed color at least 1.
  if((opponent >= colorNeeded || teammate >= colorNeeded)
     && std::abs(opponent - teammate) >= colorNeeded / 2
     && std::abs(opponent - teammate) >= 1)
  {
    if(!searchForPlayer)
    {
      return (ownJerseyIsBlack && opponent < teammate) || (opponentJerseyIsBlack && opponent > teammate);
    }
    else if(white <= (opponent + teammate) * 3)
    {
      obstacle.detectedJersey = true;
      obstacle.ownTeam = opponent < teammate;
      DRAWTEXT("module:PlayersPerceptor", obstacle.realCenterX, y - 5, 10, ColorRGBA::black, (opponent < teammate ? teammate : opponent));
      return true;
    }
  }
  return false;
}

void PlayersPerceptor::analyzePixel(int x, int y, const Image::Pixel& current, const Image::Pixel& color, unsigned char blackI,
                                    bool searchForPlayer, int& black, int& nonBlack) const
{
  if(current.i > maxBlackI)
  {
    int diffH = (int) std::min(std::min(std::abs(current.h - color.h), (std::abs(current.h - color.h + 255))), (std::abs(current.h - color.h - 255)));
    int diffS = (int) std::abs(current.s - color.s);
    if(current.i < maxDarkI)
    {
      if(diffH < maxRangeH * 2 && current.s > maxGrayS)
      {
        RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
        ++nonBlack;
      }
      else
      {
        RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, (searchForPlayer ? ColorRGBA::blue : ColorRGBA::violet));
        ++black;
      }
    }
    else if(diffH < maxRangeH && diffS < maxRangeS && current.s > maxGrayS)
    {
      RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
      nonBlack++;
    }
  }
  else
  {
    int diffI = (int) std::abs(current.i - blackI);
    if(diffI < maxRangeI)
    {
      RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, (searchForPlayer ? ColorRGBA::blue : ColorRGBA::violet));
      black++;
    }
  }
}

Vector2f PlayersPerceptor::computeVanishingPointZ() const
{
  // compute vanishing point for the world z axis by intersecting
  // that axis with the image plane
  Vector3f vAxis = theCameraMatrix.rotation.row(2);
  vAxis *= std::max(-1e6f, std::min(1e6f, theCameraInfo.focalLength / vAxis.x()));
  Vector2f vanishingPoint(vAxis.y(), vAxis.z());
  vanishingPoint = theCameraInfo.opticalCenter - vanishingPoint;
  return vanishingPoint;
}

MAKE_MODULE(PlayersPerceptor, perception)
