/**
* @file PlayersPerceptor.cpp
* @ author Michel Bartsch, Vitali Gutsch
*/

#include "PlayersPerceptor.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/ColorClasses.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ColorModelConversions.h"
#include <algorithm>

void PlayersPerceptor::update(PlayersPercept& playersPercept)
{
  DECLARE_DEBUG_DRAWING("module:PlayersPerceptor", "drawingOnImage");

  playersPercept.players.clear();

  if(!theCameraMatrix.isValid || !calcRealWorldSizes())
    return;

  for(int i = 0, x = 0, xEnd = theImage.width - xStep; x < xEnd; i++, x += xStep)
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
    horizon = 1;
    pCorrected = theImageCoordinateSystem.toCorrected(Vector2i(theImage.width / 2, horizon));
    if(!Transformation::imageToRobot(pCorrected, theCameraMatrix, theCameraInfo, point))
    {
      return false;
    }
    else if(theCameraInfo.camera == CameraInfo::lower)
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
    if(point.x() == 0 && point.y() == 0)
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
  else if(horizon >= theImage.height - 1)
  {
    return false;
  }
  else
  {
    farthestXGap = 0;
    farthestYGap = 0;
    farthestMinWidth = 0;
  }
  pCorrected = theImageCoordinateSystem.toCorrected(Vector2i(theImage.width / 2, theImage.height - 1));
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
  int yEnd = theImage.height - 1;
  theBodyContour.clipBottom(x, yEnd, yEnd);
  float grow = (y > 0 ? growBaseUpper : growBaseLower) * nearestMinWidth / std::max(theImage.height - y, 1);
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
    ASSERT(x >= 0 && x < theImage.width && y >= 0 && y < theImage.height);
    //RECTANGLE("module:PlayersPerceptor", x, y, x, y, 0, Drawings::solidPen, ColorRGBA::yellow);
    const Image::Pixel& pixel = theImage[y][x];
    if(theColorTable[pixel].is(ColorClasses::green) || theColorTable[pixel].is(ColorClasses::orange))
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
  for(int i = 0, iEnd = (theImage.width - xStep) / xStep; i < iEnd; i++)
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
  float distanceFactor = (maxY - horizon) / (float)(theImage.height - horizon);
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
  for(int rightEnd = (theImage.width - 1) / xStep; right < rightEnd; right++)
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
  if(right >= theImage.width)
  {
    right = theImage.width - 1;
  }

  // find exact bottom
  y = maxY;
  gap = 0;
  int yEnd = theImage.height;
  theBodyContour.clipBottom(maxYIndex * xStep, yEnd, yEnd);
  for(int x = maxYIndex * xStep; y < yEnd; y++)
  {
    //RECTANGLE("module:PlayersPerceptor", x, y, x, y, 0, Drawings::solidPen, ColorRGBA::yellow);
    const Image::Pixel& pixel = theImage[y][x];
    if(theColorTable[pixel].is(ColorClasses::green) || theColorTable[pixel].is(ColorClasses::orange))
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
  int y = (int)(height * (obstacle.y2 - obstacle.y1) + obstacle.y1);
  float x = (float)(obstacle.x1FeetOnly + ((obstacle.x2FeetOnly - obstacle.x1FeetOnly) >> 2));
  float xEnd = (float)(obstacle.x2FeetOnly - ((obstacle.x2FeetOnly - obstacle.x1FeetOnly) >> 2));
  float step = (xEnd - x) / jerseyScanConcentration;
  if(step < 1)
  {
    step = 1;
  }
  int opponent = 0;
  int teammate = 0;
  int white = 0;

  bool ownJerseyIsBlack = false;
  unsigned char ownColorH = 0;
  unsigned char ownColorS = 0;
  unsigned char ownColorI = 0;
  if(theOwnTeamInfo.teamColor == TEAM_BLACK)
  {
    ownJerseyIsBlack = true;
    ownColorH = 85;
    ownColorS = 255;
    ownColorI = 2;
  }
  else if(theOwnTeamInfo.teamColor == TEAM_BLUE)
  {
    ownColorH = 126;
    ownColorS = 255;
    ownColorI = 172;
  }
  else if(theOwnTeamInfo.teamColor == TEAM_RED)
  {
    ownColorH = 214;
    ownColorS = 254;
    ownColorI = 168;
  }
  else if(theOwnTeamInfo.teamColor == TEAM_YELLOW)
  {
    ownColorH = 44;
    ownColorS = 255;
    ownColorI = 172;
  }

  bool opponentsJerseyIsBlack = false;
  unsigned char opponentColorH = 0;
  unsigned char opponentColorS = 0;
  unsigned char opponentColorI = 0;
  if(theOpponentTeamInfo.teamColor == TEAM_BLUE)
  {
    opponentColorH = 126;
    opponentColorS = 255;
    opponentColorI = 172;
  }
  else if(theOpponentTeamInfo.teamColor == TEAM_RED)
  {
    opponentColorH = 214;
    opponentColorS = 254;
    opponentColorI = 168;
  }
  else if(theOpponentTeamInfo.teamColor == TEAM_BLACK)
  {
    opponentsJerseyIsBlack = true;
    opponentColorH = 85;
    opponentColorS = 255;
    opponentColorI = 2;
  }
  else if(theOpponentTeamInfo.teamColor == TEAM_YELLOW)
  {
    opponentColorH = 44;
    opponentColorS = 255;
    opponentColorI = 172;
  }

  unsigned char currH = 0;
  unsigned char currS = 0;
  unsigned char currI = 0;

  int diffToOwnH = 0;
  int diffToOwnS = 0;
  int diffToOwnI = 0;
  int diffToOpponentsH = 0;
  int diffToOpponentsS = 0;
  int diffToOpponentsI = 0;

  int colorNeeded = (int)(jerseyNeeded * jerseyScanConcentration);
  if(colorNeeded < 1)
  {
    colorNeeded = 1;
  }

  for(; x <= xEnd; x += step)
  {
    Image::Pixel currentPixel = theImage[y][(int) x];
    ColorModelConversions::fromYCbCrToHSI(currentPixel.y, currentPixel.cb, currentPixel.cr, currH, currS, currI);

    RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::yellow);

    if(currS < maxWhiteS && currI > minWhiteI)
    {
      RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::white);
      white++;
    }
    else if(currH > minGreenH && currH < maxGreenH && currI > minGreenI)
    {
      RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::green);
    }
    else if(ownJerseyIsBlack)
    {
      if(currI > maxBlackI)
      {
        diffToOpponentsH = (int) std::min(std::min(std::abs(currH - opponentColorH), (std::abs(currH - opponentColorH + 255))), (std::abs(currH - opponentColorH - 255)));
        diffToOpponentsS = (int) std::abs(currS - opponentColorS);
        if(currI < maxDarkI)
        {
          if(diffToOpponentsH < maxRangeH * 2 && currS > maxGrayS)
          {
            RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
            opponent++;
          }
          else
          {
            RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, (searchForPlayer ? ColorRGBA::blue : ColorRGBA::violet));
            teammate++;
          }
        }
        else if(diffToOpponentsH < maxRangeH && diffToOpponentsS < maxRangeS && currS > maxGrayS)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
          opponent++;
        }
      }
      else
      {
        diffToOwnI = (int) std::abs(currI - ownColorI);
        if(diffToOwnI < maxRangeI)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, (searchForPlayer ? ColorRGBA::blue : ColorRGBA::violet));
          teammate++;
        }
      }
    }
    else if(opponentsJerseyIsBlack)
    {
      if(currI > maxBlackI)
      {
        diffToOwnH = (int) std::min(std::min(std::abs(currH - ownColorH), (std::abs(currH - ownColorH + 255))), (std::abs(currH - ownColorH - 255)));
        diffToOwnS = (int) std::abs(currS - ownColorS);
        if(currI < maxDarkI)
        {
          if(diffToOwnH < maxRangeH * 2 && currS > maxGrayS)
          {
            RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
            teammate++;
          }
          else
          {
            RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, (searchForPlayer ? ColorRGBA::blue : ColorRGBA::violet));
            opponent++;
          }
        }
        else if(diffToOwnH < maxRangeH && diffToOwnS < maxRangeS && currS > maxGrayS)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
          teammate++;
        }
      }
      else
      {
        diffToOpponentsI = (int) std::abs(currI - opponentColorI);
        if(diffToOpponentsI < maxRangeI)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, (searchForPlayer ? ColorRGBA::blue : ColorRGBA::violet));
          opponent++;
        }
      }
    }
    else if(currI > maxBlackI)
    {
      diffToOwnH = (int) std::min(std::min(std::abs(currH - ownColorH), (std::abs(currH - ownColorH + 255))), (std::abs(currH - ownColorH - 255)));
      diffToOpponentsH = (int) std::min(std::min(std::abs(currH - opponentColorH), (std::abs(currH - opponentColorH + 255))), (std::abs(currH - opponentColorH - 255)));
      if(currI < maxDarkI)
      {
        // Difference of the colorchannels from the found Pixel to the own color
        if(diffToOpponentsH < maxRangeH * 2 && !(diffToOwnH < maxRangeH * 2) && currS > maxGrayS)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
          opponent++;
        }
        else if(!(diffToOpponentsH < maxRangeH * 2) && diffToOwnH < maxRangeH * 2 && currS > maxGrayS)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::blue);
          teammate++;
        }
        else if(diffToOpponentsH < maxRangeH * 2 && diffToOwnH < maxRangeH * 2 && currS > maxGrayS)
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
      else
      {
        if(diffToOpponentsH < maxRangeH && !(diffToOwnH < maxRangeH) && currS > maxGrayS)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::red);
          opponent++;
        }
        else if(!(diffToOpponentsH < maxRangeH) && diffToOwnH < maxRangeH && currS > maxGrayS)
        {
          RECTANGLE("module:PlayersPerceptor", x, y, x, y, 1, Drawings::solidPen, ColorRGBA::blue);
          teammate++;
        }
        else if(diffToOpponentsH < maxRangeH && diffToOwnH < maxRangeH && currS > maxGrayS)
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
  }

  // when there is more color of a jersey in the player as needed to
  // find a jersey and the difference between the two
  // jersey colors is more than half of the needed color at least 1.
  if((opponent >= colorNeeded || teammate >= colorNeeded)
     && abs(opponent - teammate) >= colorNeeded / 2
     && abs(opponent - teammate) >= 1)
  {
    if(!searchForPlayer)
    {
      if(ownJerseyIsBlack)
      {
        if(opponent < teammate) return true;
      }
      else if(opponentsJerseyIsBlack)
      {
        if(opponent > teammate) return true;
      }
      return false;
    }
    else if(!(white > (opponent + teammate) * 3))
    {
      obstacle.detectedJersey = true;
      obstacle.ownTeam = opponent < teammate;
      DRAWTEXT("module:PlayersPerceptor", obstacle.realCenterX, y - 5, 10, ColorRGBA::black, (opponent < teammate ? teammate : opponent));
      return true;
    }
  }
  return false;
}


MAKE_MODULE(PlayersPerceptor, perception)
