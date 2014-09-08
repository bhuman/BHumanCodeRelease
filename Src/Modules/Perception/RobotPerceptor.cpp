/**
* @file RobotPerceptor.cpp
* @ author Michel Bartsch
*/

#include "RobotPerceptor.h"
#include "Tools/ColorClasses.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>

void RobotPerceptor::update(RobotPercept& robotPercept)
{
  DECLARE_DEBUG_DRAWING("module:RobotPerceptor", "drawingOnImage");

  robotPercept.robots.clear();

  if(!theCameraMatrix.isValid || !calcRealWorldSizes())
    return;

  for(int i = 0, x = 0, xEnd = theImage.width - xStep; x < xEnd; i++, x += xStep)
  {
    scanVerticalLine(i, x);
    if(verticalLines[i].y != 0)
    {
      CROSS("module:RobotPerceptor", x, verticalLines[i].y, 1, 1, Drawings::ps_solid, ColorRGBA::orange);
    }
  }

  while(combineVerticalLines(robotPercept));

  for(auto i = robotPercept.robots.begin(), end = robotPercept.robots.end(); i != end; i++)
  {
    if(theCameraInfo.camera)
    {
      scanJersey(*i, 0);
    }
    else
    {
      if(!scanJersey(*i, jerseyYFirst))
      {
        scanJersey(*i, jerseyYSecond);
      }
    }
  }
}

bool RobotPerceptor::calcRealWorldSizes()
{
  horizon = (int)theImageCoordinateSystem.origin.y;
  Vector2<float> pCorrected;
  Vector2<float> point;
  if(horizon < 1)
  {
    horizon = 1;
    pCorrected = theImageCoordinateSystem.toCorrected(Vector2<int>(theImage.width / 2, horizon));
    Transformation::imageToRobot(pCorrected.x, pCorrected.y, theCameraMatrix, theCameraInfo, point);
    if(theCameraInfo.camera)
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
    if(point.x == 0 && point.y == 0)
    {
      farthestXGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestXGap, (float)minFarthestMillis);
      farthestYGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestYGap, (float)minFarthestMillis);
      farthestMinWidth = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestMinWidth, (float)minFarthestMillis);
    }
    else
    {
      farthestXGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestXGap, point.abs());
      farthestYGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestYGap, point.abs());
      farthestMinWidth = (int)Geometry::getSizeByDistance(theCameraInfo, (float)farthestMinWidth, point.abs());
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
  pCorrected = theImageCoordinateSystem.toCorrected(Vector2<int>(theImage.width / 2, theImage.height - 1));
  Transformation::imageToRobot(pCorrected.x, pCorrected.y, theCameraMatrix, theCameraInfo, point);
  nearestXGap = (theCameraInfo.camera ? xGapMillisLower : xGapMillisUpper);
  nearestXGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)nearestXGap, point.abs());
  nearestYGap = (theCameraInfo.camera ? yGapMillisLower : yGapMillisUpper);
  nearestYGap = (int)Geometry::getSizeByDistance(theCameraInfo, (float)nearestYGap, point.abs());
  nearestMinWidth = (theCameraInfo.camera ? minWidthMillisLower : minWidthMillisUpper);
  nearestMinWidth = (int)Geometry::getSizeByDistance(theCameraInfo, (float)nearestMinWidth, point.abs());
  if(nearestMinWidth < 0)
  {
    //camera was looking very high,
    //the lowest point was too far away and produced overflow.
    //Because of the earth curvature, this point most likely never touches the earth.
    // -> Remove this for perception of mars rovers ;)
    return false;
  }

  return true;
}

void RobotPerceptor::scanVerticalLine(int index, int x)
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
    //RECTANGLE("module:RobotPerceptor", x, y, x, y, 0, Drawings::ps_solid, ColorRGBA::yellow);
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
        much better performance but problems with fallen robots
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

bool RobotPerceptor::combineVerticalLines(RobotPercept& robotPercept)
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
    //RECTANGLE("module:RobotPerceptor", x, y, x, y, 0, Drawings::ps_solid, ColorRGBA::yellow);
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
  RobotPercept::RobotBox obstacle = RobotPercept::RobotBox();
  obstacle.x1 = left;
  obstacle.x2 = right;
  obstacle.y1 = top;
  obstacle.y2 = y;
  obstacle.realCenterX = realCenterX;
  obstacle.x1FeetOnly = feetLeft;
  obstacle.x2FeetOnly = feetRight;
  obstacle.lowerCamera = theCameraInfo.camera == theCameraInfo.lower;
  obstacle.detectedJersey = false;
  obstacle.detectedFeet = !verticalLines[maxYIndex].noFeet;
  obstacle.teamRed = false;
  obstacle.fallen = (onGroundSum >= ((right - left) / xStep) * onGroundMajority);
  robotPercept.robots.push_back(obstacle);

  return true;
}

bool RobotPerceptor::scanJersey(RobotPercept::RobotBox& obstacle, float height)
{
  int y = (int)(height * (obstacle.y2 - obstacle.y1) + obstacle.y1);
  float x = (float)(obstacle.x1 + ((obstacle.x2 - obstacle.x1) >> 2));
  float xEnd = (float)(obstacle.x2 - ((obstacle.x2 - obstacle.x1) >> 2));
  float step = (xEnd - x) / jerseyScanConcentration;
  if(step < 1)
  {
    step = 1;
  }
  int magenta = 0;
  int cyan = 0;
  int colorNeeded = (int)(jerseyNeeded * jerseyScanConcentration);
  if(colorNeeded < 1)
  {
    colorNeeded = 1;
  }

  for(; x <= xEnd; x += step)
  {
    //RECTANGLE("module:RobotPerceptor", x, y, x, y, 0, Drawings::ps_solid, ColorRGBA::yellow);
    const ColorTable::Colors colors = theColorTable[theImage[y][(int)x]];
    if(colors.is(ColorClasses::red) || colors.is(ColorClasses::orange))
    {
      magenta++;
    }
    if(colors.is(ColorClasses::blue))
    {
      cyan++;
    }
  }

  // when there is more color of a jersey in the robot as needed to
  // find a jersey and the difference between the two
  // jersey colors is more than half of the needed color at least 1.
  if((magenta >= colorNeeded || cyan >= colorNeeded)
     && abs(magenta - cyan) >= colorNeeded / 2
     && abs(magenta - cyan) >= 1)
  {
    obstacle.detectedJersey = true;
    obstacle.teamRed = magenta > cyan;
    return true;
  }
  return false;
}

MAKE_MODULE(RobotPerceptor, Perception)