/**
 * The file declares a module that creates colored regions on a number of scanlines.
 * Although the scanlines are vertical, scanning is actually performed horizontally
 * by advancing all vertical scanlines in parallel from the image bottom to the top.
 * The upper bound of all scanlines is the upper image border and the horizon.
 * The lower bound is the lower image border and the body contour.
 * The module is based on Arne Böckmann's original implementation of this idea.
 * @author Thomas Röfer
 */

#include "ScanlineRegionizer.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>

MAKE_MODULE(ScanlineRegionizer, Perception)

void ScanlineRegionizer::update(ScanlineRegions& regions)
{
  DECLARE_DEBUG_DRAWING("module:ScanlineRegionizer:grid", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanlineRegionizer:edges", "drawingOnImage");

  regions.scanlines.clear();

  if(!theCameraMatrix.isValid)
    return; // Cannot compute grid without camera matrix

  const int horizon = std::max(static_cast<int>(theImageCoordinateSystem.origin.y), -1);
  if(horizon >= theImage.height)
    return; // Image is above horizon -> no grid in image

  Vector2<> pointOnField;
  if(!Transformation::imageToRobotWithCameraRotation(Vector2<int>(theImage.width / 2, theImage.height - 1), theCameraMatrix, theCameraInfo, pointOnField))
    return; // Cannot project lower image border to field -> no grid

  // Initialize the scan states and the regions
  scanStates.clear();
  const int xStep = theImage.width / numOfScanlines;
  const int xStart = theImage.width % (theImage.width / xStep - 1) / 2;
  const Image::Pixel* pImg = &theImage[theImage.height - 1][xStart];
  for(int x = xStart; x < theImage.width; x += xStep, pImg += xStep)
  {
    int upperBodyLimit = theImage.height;
    theBodyContour.clipBottom(x, upperBodyLimit);
    scanStates.emplace_back(theImage.height, theColorTable[*pImg], upperBodyLimit);
    regions.scanlines.emplace_back(x);
  }

  // Perform the actual scan on all scanlines.
  const float fieldStep = theFieldDimensions.fieldLinesWidth * lineWidthRatio;
  bool singleSteps = false;
  Vector2<> pointInImage;
  for(int y = theImage.height - 1;;)
  {
    const int y2 = y;

    // Calc next vertical position for all scanlines.
    if(singleSteps)
      --y;
    else
    {
      pointOnField.x += fieldStep;
      if(!Transformation::robotWithCameraRotationToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
        break;
      y = std::min(y2 - 1, static_cast<int>(pointInImage.y + 0.5));
      singleSteps = y2 - 1 == y;
    }

    // When the horizon is reached, we are done.
    if(y <= horizon)
      break;

    // Advance all scanlines by one row.
    const int otherColorThreshold = std::max(static_cast<int>(minColorRatio * (y2 - y)), 1);
    const Image::Pixel* pImg = &theImage[y][xStart];
    auto scanline = regions.scanlines.begin();
    for(auto scanState = scanStates.begin(); scanState != scanStates.end(); ++scanState, ++scanline, pImg += xStep)
    {
      const int x = scanline->x;
      if(y < scanState->upperBodyLimit)
      {
        LINE("module:ScanlineRegionizer:grid", x, y - 1, x, y, 2, Drawings::ps_solid, ColorRGBA::blue);

        // If we leave the body contour, start a new (first) segment on this scanline
        if(scanState->isBelowBodyLimit)
        {
          scanState->isBelowBodyLimit = false;
          scanState->y = scanState->upperBodyLimit;
          scanState->color = theColorTable[theImage[scanState->upperBodyLimit - 1][x]];
        }

        // If color changes, determine edge position between last and current scanpoint
        const ColorTable::Colors& color = theColorTable[*pImg];
        if(color.colors != scanState->color.colors)
        {
          const int yMin = std::max(y - otherColorThreshold + 1, 0);
          int counter = 0;
          int yy = std::min(y2 - 1, scanState->upperBodyLimit - 1);
          for(const Image::Pixel* pImg2 = &theImage[yy][x]; yy >= yMin && counter < otherColorThreshold; --yy, pImg2 -= theImage.widthStep)
            if(theColorTable[*pImg2].colors != scanState->color.colors)
              ++counter;
            else
              counter = 0;

          // Enough pixels of different colors were found: end previous region and start a new one.
          if(counter == otherColorThreshold)
          {
            yy += otherColorThreshold + 1;
            ASSERT(scanState->y > yy);
            scanline->regions.emplace_back(scanState->y, yy, scanState->color);
            scanState->color = color;
            scanState->y = yy;
            CROSS("module:ScanlineRegionizer:edges", x, yy, 2, 1, Drawings::ps_solid, ColorRGBA::red);
          }
        }
      }
    }
  }

  // End all regions
  auto scanline = regions.scanlines.begin();
  for(auto scanState = scanStates.begin(); scanState != scanStates.end(); ++scanState, ++scanline)
    if(!scanState->isBelowBodyLimit)
    {
      ASSERT(scanState->y > horizon + 1);
      scanline->regions.emplace_back(scanState->y, horizon + 1, scanState->color);
    }
}
