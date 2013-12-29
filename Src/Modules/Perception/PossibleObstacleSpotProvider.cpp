/*
 * LineSpotProviderExp.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#include "PossibleObstacleSpotProvider.h"
#include <vector>
#include <deque>
#include <algorithm>
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Perception/PossibleObstacleSpots.h"

using namespace std;

PossibleObstacleSpotProvider::PossibleObstacleSpotProvider()
{
  gridY.reserve(maxResolutionHeight);
  crossSizes.reserve(maxResolutionHeight);
  nonGreenCount.resize(maxResolutionHeight);
  minNonGreenCounts.reserve(maxResolutionHeight);
  for(int i = 0; i < maxResolutionHeight; ++i)
  {
    nonGreenCount[i].resize(maxResolutionWidth);
  }
}

void PossibleObstacleSpotProvider::update(PossibleObstacleSpots & obs)
{
  DECLARE_DEBUG_DRAWING("module:PossibleObstacleSpotProvider:grid", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PossibleObstacleSpotProvider:cross", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PossibleObstacleSpotProvider:earlyAbort", "drawingOnImage");
  obs.clear();
  if(!theCameraMatrix.isValid) return;
  if(theMotionInfo.motion != MotionInfo::stand &&
     theMotionInfo.motion != MotionInfo::walk) return;

  //do nothing if we are looking over our own shoulder using the lower camera.
  if(theCameraInfo.camera == CameraInfo::lower && abs(theCameraMatrix.rotation.getZAngle()) >= lowerCameraMaxAngle) return;
  calculateGrid();
  STOP_TIME_ON_REQUEST("PossibleObstacleSpotProvider:sumImage", calculateNonGreenRowCount(););

  const int scanlineDistVertPixel = calculateScanlineDistance(theImage.width, scanLineDist);
  int startX = scanlineDistVertPixel/2 > 1 ? scanlineDistVertPixel/2 : 1;
  int endX = scanlineDistVertPixel/2 > 1 ? theImage.width : theImage.width-1;
  int heightMinus1 = theImage.height - 1;
  const int horizon = calcHorizon();
  //skip leftmost scanline
  startX += scanlineDistVertPixel;
  //skip rightmost scanline
  endX -= scanlineDistVertPixel;
  for(int x = startX; x < endX; x += scanlineDistVertPixel)
  {
    PossibleObstacleSpots::Scanline& line = obs.getNextScanline();
    line.clear();
    line.xImg = x;
    int y = heightMinus1;
    theBodyContour.clipBottom(x, y, theImage.height);
    int yMin = horizon;

    if(theCameraInfo.camera == CameraInfo::upper)
    { //only use field boundary in upper cam image
      //We cannot use the field border in the lower cam because it is always directly in front
      //of the robot if an obstacle is very close to the robot
      yMin = std::max(horizon, theFieldBoundary.getBoundaryY(x));
    }
    unsigned i = 0;

    if(!getFirstIndexAboveBodyConture(x, i))
    {//this happens if no grid points are above the body contour
      continue;
    }

    //A spot needs to have least n neighbors above him
    //the buffer is used to make sure that this constraint is fulfilled
    std::deque<int> bufferedY;
    for(; i < gridY.size(); ++i)
    {
      const int y = gridY[i];
      const int crossSize = crossSizes[i];

      if(y <= yMin)
      {//hit the field boundary
        break;
      }

      if(isNotGreenEnough(x, i, crossSize))
      {
        bufferedY.push_front(y);
      }
      else
      {
        bufferedY.clear();
      }

      if(bufferedY.size() >= minNumOfSpotsInARow)
      {
        line.spots[line.spotCount] = bufferedY.back();
        bufferedY.pop_back();
        ++line.spotCount;
      }

       CROSS("module:PossibleObstacleSpotProvider:grid", x, y,
            1, 1, Drawings::ps_solid, ColorClasses::blue);
      //we don't want to check too many pixels
    }
  }
}

void PossibleObstacleSpotProvider::calculateGrid()
{
  const int horizon = std::max(calcHorizon(), theFieldBoundary.highestPoint.y);

  gridY.clear();
  crossSizes.clear();
  minNonGreenCounts.clear();
  int y = theImage.height - 1;
  const int x = theImage.width / 2;
  while (y > horizon)
  {
    gridY.push_back(y);
    int lineWidth = Geometry::calculateLineSize(x, y, theCameraMatrix, theCameraInfo,
                                                      theFieldDimensions.fieldLinesWidth);
    lineWidth = std::max(minCrossSize, lineWidth); //near the horizon the width will become 0 resulting in an endless loop
    crossSizes.push_back(lineWidth);
    y -= std::max(2*lineWidth, minGridJump);
    minNonGreenCounts.push_back(int((lineWidth * 4 - 3) * minNonGreen));
  }
}

int PossibleObstacleSpotProvider::calcHorizon() const
{
  int horizon = static_cast<int>(theImageCoordinateSystem.origin.y);
  if(horizon < 0)
  {
    horizon = 0;
  }
  else if(horizon > theImage.height)
  {
    horizon = theImage.height;
  }
  return horizon;
}

bool PossibleObstacleSpotProvider::isNotGreenEnough(const int x, const int gridYIndex, const int width) const
{
  const int minNonGreen = minNonGreenCounts[gridYIndex];
  const int y = gridY[gridYIndex];
  int nonGreenCnt = 0;

  int leftX = x - width;
  if(leftX < 0)
  {
    return false;
  }
  const int rightX = x + width;
  if(rightX >= theImage.width)
  {
    return false;
  }
  int topY = y - width;
  if(topY < 0)
  {
    return false;
  }
  const int bottomY = y + width;
  if(bottomY >= theImage.height)
  {
    return false;
  }

  //left to right pixel summing is done using the pre calculated non green sums
  nonGreenCnt += nonGreenCount[gridYIndex][rightX] - nonGreenCount[gridYIndex][leftX];
  ASSERT(nonGreenCnt >= 0);
  ASSERT(minNonGreen >= nonGreenCnt);
  
  LINE("module:PossibleObstacleSpotProvider:cross", leftX,
      y, rightX, y, 2, Drawings::ps_solid, ColorRGBA(255,0,0,100));
  LINE("module:PossibleObstacleSpotProvider:cross", x,
      topY, x, bottomY, 2, Drawings::ps_solid, ColorRGBA(255,0,0,100)); 
  //check if it is still possible to reach the minNonGreenCount or not. If not abort to safe performance
  if(minNonGreen - nonGreenCnt > 2*width - 1)
  {
    CROSS("module:PossibleObstacleSpotProvider:earlyAbort", x, y,
            1, 1, Drawings::ps_solid, ColorClasses::yellow);
    return false;
  }

  //top to bottom pixel summing does not use pre calculation because the filter does not overlap vertically
  const Image::Pixel* pPixel = &theImage[topY][x];
  const Image::Pixel* pEnd = &theImage[bottomY][x];
  for(;pPixel <= pEnd; pPixel += theImage.widthStep)
  {
    if(!theColorReference.isGreen(pPixel))
    {
      ++nonGreenCnt;
    }
  }
  return nonGreenCnt >= minNonGreen;
}

int PossibleObstacleSpotProvider::calculateScanlineDistance(int size, float percent) const
{
  const int pixelsBetweenScanlines = (int) ((size * percent) / 100.0);
  int scanlineDistance = pixelsBetweenScanlines + 1;

  while(size % scanlineDistance != 0)
  {
    scanlineDistance++;
  }
  return scanlineDistance;
}

bool PossibleObstacleSpotProvider::getFirstIndexAboveBodyConture(const int x, unsigned& outIndex) const
{
  int y = theImage.height - 1;
  theBodyContour.clipBottom(x, y);
  for(int i = 0; i < (int) gridY.size() - 1; ++i)
  {
    if(gridY[i] <= y)
    {
      outIndex = i + 1;
      return true;
    }
  }
  return false;
}

void PossibleObstacleSpotProvider::calculateNonGreenRowCount()
{

  for(unsigned i = 0; i < gridY.size(); ++i)
  {
    const int y = gridY[i];
    const Image::Pixel* pPixel = &theImage[y][0];
    const Image::Pixel* pEnd = &theImage[y][theImage.width-1];
    nonGreenCount[i].clear();
    int x = 0;
    int sum = 0;
    for(; pPixel <= pEnd; ++pPixel, ++x)
    {
      if(!theColorReference.isGreen(pPixel))
      {
        ++sum;
      }
      nonGreenCount[i].push_back(sum);
    }
  }
}

MAKE_MODULE(PossibleObstacleSpotProvider, Perception)
