/**
 * @file ScanLineRegionizer.cpp
 *
 * This file implements a module that segments the image horizontally
 * and vertically by detecting edges and applying heuristics to classify
 * the regions between them.
 *
 * @author Lukas Malte Monnerjahn
 * @author Arne Hasselbring
 */

#include "ScanLineRegionizer.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Debugging/Annotation.h"

#include <functional>
#include <list>
#include <vector>

MAKE_MODULE(ScanLineRegionizer);

void ScanLineRegionizer::update(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal)
{
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:horizontalRegionSplit", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:horizontalRegionUnion", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:fieldColorRangeHorizontal", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:whiteRange", "drawingOnImage");

  colorScanLineRegionsHorizontal.scanLines.clear();

  if(!theScanGrid.isValid() || !theFieldBoundary.isValid)
    return;

  // 0. Preprocess
  approximateBaseLuminance();
  approximateBaseSaturation();

  // 1. Define scan-lines
  std::vector<unsigned short> yPerScanLine;  // heights of the scan-lines in the image
  std::vector<std::vector<InternalRegion>> regionsPerScanLine;

  // find height in image up to which additional smoothing (5x5 filter) will be applied.
  // compute by projecting on-field distance into image
  Vector2f pointInImage;  // helper variable for projection
  const int middle = theCameraInfo.camera == CameraInfo::lower ? 0 :
                     Transformation::robotWithCameraRotationToImage(additionalSmoothingPoint, theCameraMatrix, theCameraInfo, pointInImage) ?
                     static_cast<const int>(pointInImage.y()) : theCameraInfo.height - 1;
  LINE("module:ScanLineRegionizer:horizontalRegionSplit", 0, middle, theECImage.grayscaled.width, middle, 3, Drawings::PenStyle::dottedPen, ColorRGBA(80, 6, 80));

  for(const ScanGrid::HorizontalLine& horizontalLine : theScanGrid.lowResHorizontalLines)
  {
    yPerScanLine.emplace_back(horizontalLine.y);
    regionsPerScanLine.emplace_back();

    // 2. Detect edges and create temporary regions in between including a representative YHS triple.
    if(theCameraInfo.camera == CameraInfo::lower || middle < horizontalLine.y)
    {
      scanHorizontalAdditionalSmoothing(horizontalLine.y, regionsPerScanLine.back(), horizontalLine.left, horizontalLine.right);
    }
    else
    {
      scanHorizontal(horizontalLine.y, regionsPerScanLine.back(), horizontalLine.left, horizontalLine.right);
    }
  }

  // 3. Classify field regions.
  uniteHorizontalFieldRegions(yPerScanLine, regionsPerScanLine);
  classifyFieldRegions(yPerScanLine, regionsPerScanLine, true);

  // 4. Classify white regions.
  classifyWhiteRegionsWithThreshold(regionsPerScanLine);

  // 5. Fill small gaps between field and white regions
  stitchUpHoles(regionsPerScanLine, true);

  // 6. Convert to external representation.
  emplaceInScanLineRegionsHorizontal(colorScanLineRegionsHorizontal, yPerScanLine, regionsPerScanLine);
}

void ScanLineRegionizer::update(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped)
{
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:verticalRegionSplit", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:verticalRegionUnion", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:fieldColorRangeVertical", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:whiteRange", "drawingOnImage");

  colorScanLineRegionsVerticalClipped.scanLines.clear();

  if(theScanGrid.verticalLines.empty() || theScanGrid.lowResHorizontalLines.empty() || !theFieldBoundary.isValid)
    return;

  colorScanLineRegionsVerticalClipped.lowResStart = theScanGrid.lowResStart;
  colorScanLineRegionsVerticalClipped.lowResStep = theScanGrid.lowResStep;

  // 0. Preprocess
  approximateBaseLuminance();
  approximateBaseSaturation();

  // 1. Define scan-lines and limit scan-line ranges by field boundary (body contour is already excluded by ScanGrid)
  const std::size_t numOfScanLines = theScanGrid.verticalLines.size();
  std::vector<unsigned short> xPerScanLine(numOfScanLines);
  std::vector<std::vector<InternalRegion>> regionsPerScanLine(numOfScanLines);

  // find height in image up to which additional smoothing (5x5 filter) will be applied.
  // compute by projecting on-field distance into image
  Vector2f pointInImage;  // helper variable for projection
  const int middle = theCameraInfo.camera == CameraInfo::lower ? 0 :
                     Transformation::robotWithCameraRotationToImage(additionalSmoothingPoint, theCameraMatrix, theCameraInfo, pointInImage) ?
                     static_cast<const int>(pointInImage.y()) : theCameraInfo.height - 1;
  LINE("module:ScanLineRegionizer:verticalRegionSplit", 0, middle, theECImage.grayscaled.width, middle, 3, Drawings::PenStyle::dottedPen, ColorRGBA(80, 6, 80));

  for(std::size_t i = 0; i < theScanGrid.verticalLines.size(); ++i)
  {
    xPerScanLine[i] = static_cast<unsigned short>(theScanGrid.verticalLines[i].x);
    const int top = theScanGrid.verticalLines[i].yMin + 1;

    // 2. Detect edges and create temporary regions in between including a representative YHS triple.
    scanVertical(theScanGrid.verticalLines[i], middle, top, regionsPerScanLine[i]);
  }

  // 3. Classify field regions.
  uniteVerticalFieldRegions(xPerScanLine, regionsPerScanLine);
  classifyFieldRegions(xPerScanLine, regionsPerScanLine, false);

  // 4. Classify white regions.
  classifyWhiteRegionsWithThreshold(regionsPerScanLine);

  // 5. Fill small gaps between field and white regions
  stitchUpHoles(regionsPerScanLine, false);

  // 6. Convert to external representation.
  emplaceInScanLineRegionsVertical(colorScanLineRegionsVerticalClipped, xPerScanLine, regionsPerScanLine);
}

void ScanLineRegionizer::scanHorizontal(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const
{
  if(y < 1 || y >= theECImage.grayscaled.height - 1)
    return;
  // initialize variables
  const int scanY = static_cast<int>(y);
  const unsigned int scanStart = leftmostX < theECImage.grayscaled.width - 2 ? leftmostX : theECImage.grayscaled.width - 3;
  bool nextRegionWhite = false;

  ScanRun<3> scanRun(true, static_cast<int>(y), scanStart, rightmostX);
  scanRun.leftScanEdgePosition = leftmostX;

  // define filter:
  // vertical 1D gauss/sobel smoothing: filter-matrix [[1], [2], [1]]
  scanRun.gaussV = [](const PixelTypes::GrayscaledPixel* line, const unsigned int imageWidth)
  {
    return static_cast<int>(line[-static_cast<int>(imageWidth)] + 2 * line[0] + line[imageWidth]);
  };
  // horizontal 1D Gauss/sobel smoothing: filter-matrix [[1, 2, 1]]
  // results in 2D-Gaussian-filter when applied on values returned by the other 1D-Gaussian-filter
  scanRun.gaussSecond = [](std::array<int, 3>& gaussBuffer, int x)
  {
    return gaussBuffer[(x - 1) % 3] + 2 * gaussBuffer[x % 3] + gaussBuffer[(x + 1) % 3];
  };
  // linear gradient, results in sobel filter when applied on values returned by the 1D-Gaussian-filter
  scanRun.gradient = [](std::array<int, 3>& gaussBuffer, int x)
  {
    return gaussBuffer[(x - 1) % 3] - gaussBuffer[(x + 1) % 3];
  };
  // 1D 3x3 gauss filter: 1, 2, 1 -> sum: 4, sum of 2D gauss filter values is 16
  const int thresholdAdaption = 16;
  int threshold = thresholdAdaption * static_cast<int>(edgeThreshold);

  // Initialize the buffer of smoothed values.
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[scanY][scanStart];
  scanRun.leftGaussBuffer[0] = scanRun.gaussV(luminance, theECImage.grayscaled.width);
  scanRun.leftGaussBuffer[1] = scanRun.gaussV(++luminance, theECImage.grayscaled.width);
  scanRun.leftGaussBuffer[2] = scanRun.gaussV(++luminance, theECImage.grayscaled.width);

  // grid scan stuff
  unsigned int gridX = scanStart + 1;
  int gridValue = scanRun.gaussSecond(scanRun.leftGaussBuffer, 1);
  int nextGridValue;
  size_t gridLineIndex = theScanGrid.lowResStart;
  unsigned int nextGridX = theScanGrid.verticalLines[gridLineIndex].x;
  while(nextGridX <= gridX && nextGridX < rightmostX && ++gridLineIndex < theScanGrid.verticalLines.size())
  {
    nextGridX = theScanGrid.verticalLines[gridLineIndex].x;
  }
  if(nextGridX <= gridX && nextGridX < rightmostX)
    nextGridX = rightmostX; // skip the loop, make only a single region

  // 'gridLineIndex <= theScanGrid.lines.size()' so that the image's right edge becomes the last grid point
  while(gridLineIndex <= theScanGrid.verticalLines.size() && nextGridX < rightmostX)
  {
    bool regionAdded = false;
    scanRun.rightGaussBuffer[0] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX - 1], theECImage.grayscaled.width);
    scanRun.rightGaussBuffer[1] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX], theECImage.grayscaled.width);
    scanRun.rightGaussBuffer[2] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX + 1], theECImage.grayscaled.width);

    nextGridValue = scanRun.gaussSecond(scanRun.rightGaussBuffer, 1);
    if(gridValue - nextGridValue >= threshold)  // bright to dark transition
    {
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, true);
      regionAdded = true;
      if(prelabelAsWhite && nextRegionWhite && prelabelWhiteCheck(regions.back()) &&
         prelabelWhiteNeighborCheck(regions.back(), regions[regions.size() - 2]))
      {
        regions.back().color = ScanLineRegion::white;
      }
      nextRegionWhite = false;
    }
    else if(gridValue - nextGridValue <= -threshold)  // dark to bright transition
    {
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, false);
      regionAdded = true;
      nextRegionWhite = true;
    }
    if(prelabelAsWhite && regionAdded && regions.size() >= 2 && regions[regions.size() - 2].color == ScanLineRegion::white)
    {
      if(!prelabelWhiteNeighborCheck(regions[regions.size() - 2], regions.back()))
        regions[regions.size() - 2].color = ScanLineRegion::unset;
    }
    // prepare next loop
    ++gridLineIndex;
    gridX = nextGridX;
    if(gridLineIndex < theScanGrid.verticalLines.size())
      nextGridX = theScanGrid.verticalLines[gridLineIndex].x;
    else
      nextGridX = theECImage.grayscaled.width - 2;
    gridValue = nextGridValue;
    scanRun.leftGaussBuffer = scanRun.rightGaussBuffer;
  }
  // add last region
  ASSERT(scanRun.leftScanEdgePosition < rightmostX);
  regions.emplace_back(scanRun.leftScanEdgePosition, rightmostX,
                       getHorizontalRepresentativeValue(theECImage.grayscaled, scanRun.leftScanEdgePosition, rightmostX, y),
                       getHorizontalRepresentativeHueValue(theECImage.hued, scanRun.leftScanEdgePosition, rightmostX, y),
                       getHorizontalRepresentativeValue(theECImage.saturated, scanRun.leftScanEdgePosition, rightmostX, y));
}

void ScanLineRegionizer::scanHorizontalAdditionalSmoothing(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const
{
  if(y < 2 || y >= theECImage.grayscaled.height - 2)
    return;
  // initialize variables
  const int scanY = static_cast<int>(y);
  const unsigned int scanStart = leftmostX < theECImage.grayscaled.width - 4 ? leftmostX : theECImage.grayscaled.width - 5;
  const unsigned int scanStop = rightmostX >= 2 ? rightmostX - 2 : 0;

  ScanRun<5> scanRun(true, static_cast<int>(y), scanStart, rightmostX);
  scanRun.leftScanEdgePosition = leftmostX;

  // define filter
  // vertical 1D gauss/sobel smoothing: filter-matrix [[1], [2], [4], [2], [1]]
  scanRun.gaussV = [](const PixelTypes::GrayscaledPixel* line, const unsigned int imageWidth) // 5x5 gauss smoothing vertical
  {
    return static_cast<int>(line[-2 * static_cast<int>(imageWidth)] + 2 * line[-static_cast<int>(imageWidth)] + 4 * line[0] +
                            2 * line[imageWidth] + line[2 * imageWidth]);
  };
  // horizontal 1D Gauss/sobel smoothing: filter-matrix [[1, 2, 4, 2, 1]]
  // results in 2D-Gaussian-filter when applied on values returned by the other 1D-Gaussian-filter
  scanRun.gaussSecond = [](std::array<int, 5>& gaussBuffer, int x) // 5x5 gauss smoothing horizontal
  {
    return gaussBuffer[(x - 2) % 5] + 2 * gaussBuffer[(x - 1) % 5] +
           4 * gaussBuffer[x % 5] + 2 * gaussBuffer[(x + 1) % 5] + gaussBuffer[(x + 2) % 5];
  };
  // sobel-like gradient, results in sobel-like filter when applied on values returned by the 1D-Gaussian-filter
  scanRun.gradient = [](std::array<int, 5>& gaussBuffer, int x)
  {
    return gaussBuffer[(x - 2) % 5] + 2 * gaussBuffer[(x - 1) % 5] - 2 * gaussBuffer[(x + 1) % 5] - gaussBuffer[(x + 2) % 5];
  };
  // 1D 5x5 gauss filter: 1, 2, 4, 2, 1 -> sum: 10, sum of 2D gauss filter values is 100
  const int thresholdAdaption = 100;
  int threshold = thresholdAdaption * static_cast<int>(edgeThreshold);

  // Initialize the buffer of smoothed values.
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[scanY][scanStart];
  scanRun.leftGaussBuffer[0] = scanRun.gaussV(luminance, theECImage.grayscaled.width);
  scanRun.leftGaussBuffer[1] = scanRun.gaussV(++luminance, theECImage.grayscaled.width);
  scanRun.leftGaussBuffer[2] = scanRun.gaussV(++luminance, theECImage.grayscaled.width);
  scanRun.leftGaussBuffer[3] = scanRun.gaussV(++luminance, theECImage.grayscaled.width);
  scanRun.leftGaussBuffer[4] = scanRun.gaussV(++luminance, theECImage.grayscaled.width);

  // setup grid scan
  unsigned int gridX = scanStart + 2;
  int gridValue = scanRun.gaussSecond(scanRun.leftGaussBuffer, 2);
  int nextGridValue;
  size_t gridLineIndex = theScanGrid.lowResStart;
  unsigned int nextGridX;
  do
  {
    nextGridX = theScanGrid.verticalLines[gridLineIndex].x;
  }
  while(nextGridX <= gridX && nextGridX <= scanStop && ++gridLineIndex < theScanGrid.verticalLines.size());
  if(nextGridX <= gridX)
    nextGridX = rightmostX; // skip the loop, make only a single region

  while(gridLineIndex <= theScanGrid.verticalLines.size() && nextGridX <= scanStop)
  {
    scanRun.rightGaussBuffer[0] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX - 2], theECImage.grayscaled.width);
    scanRun.rightGaussBuffer[1] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX - 1], theECImage.grayscaled.width);
    scanRun.rightGaussBuffer[2] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX], theECImage.grayscaled.width);
    scanRun.rightGaussBuffer[3] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX + 1], theECImage.grayscaled.width);
    scanRun.rightGaussBuffer[4] = scanRun.gaussV(&theECImage.grayscaled[scanY][nextGridX + 2], theECImage.grayscaled.width);

    nextGridValue = scanRun.gaussSecond(scanRun.rightGaussBuffer, 2);
    if(gridValue - nextGridValue >= threshold)
    {
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, true);
    }
    else if(gridValue - nextGridValue <= -threshold)
    {
      findEdgeInSubLineHorizontal(regions, scanRun, gridX, nextGridX, false);
    }
    // prepare next loop
    gridLineIndex += theScanGrid.lowResStep;
    gridX = nextGridX;
    if(gridLineIndex < theScanGrid.verticalLines.size())
      nextGridX = theScanGrid.verticalLines[gridLineIndex].x;
    else
      nextGridX = theECImage.grayscaled.width - 3;
    gridValue = nextGridValue;
    scanRun.leftGaussBuffer = scanRun.rightGaussBuffer;
  }
  // add last region
  ASSERT(scanRun.leftScanEdgePosition < rightmostX);
  regions.emplace_back(scanRun.leftScanEdgePosition, rightmostX,
                       getHorizontalRepresentativeValue(theECImage.grayscaled, scanRun.leftScanEdgePosition, rightmostX, y),
                       getHorizontalRepresentativeHueValue(theECImage.hued, scanRun.leftScanEdgePosition, rightmostX, y),
                       getHorizontalRepresentativeValue(theECImage.saturated, scanRun.leftScanEdgePosition, rightmostX, y));
}

void ScanLineRegionizer::scanVertical(const ScanGrid::Line& line, int middle, int top, std::vector<InternalRegion>& regions) const
{
  if(line.x < 1 || static_cast<unsigned int>(line.x + 1) >= theECImage.grayscaled.width || line.yMax <= std::max(2, top))
    return;
  // initialize variables shared between 5x5 and 3x3
  unsigned int lowerY = line.yMax;
  bool nextRegionWhite = false;
  int threshold = static_cast<int>(100.f * edgeThreshold); // "100 *" because it relates to the smoothed values of the 5x5 filter.
  // y-coordinate ranges
  ASSERT(top >= 0); // 3x3 filter stop exclusive
  const int lowestY = std::min<int>(line.yMax - 2, static_cast<int>(theECImage.grayscaled.height) - 3); // 5x5 filter start
  const int middleY = std::min(std::max<int>(middle + 1, top + 1), lowestY + 1); // 5x5 filter stop exclusive, 3x3 filter start
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[lowestY][line.x];
  // grid variables
  int gridY = std::max(lowestY, middleY);
  int gridValue = 0;
  int nextGridValue;
  size_t gridYIndex = line.yMaxIndex;
  while(theScanGrid.fullResY[gridYIndex] + verticalGridScanMinStep > gridY && gridYIndex + 1 < theScanGrid.fullResY.size())
    ++gridYIndex;
  int nextGridY = theScanGrid.fullResY[gridYIndex];
  if(nextGridY >= gridY)
    nextGridY = 2;

  // start with 5x5 filter
  if(lowestY > middleY)
  {
    ScanRun<5> scanRun(false, static_cast<int>(line.x), lowestY, middleY);
    scanRun.lowerScanEdgePosition = lowerY;

    scanRun.gaussH = [](const PixelTypes::GrayscaledPixel* line) // 5x5 gauss horizontal
    {
      return static_cast<int>(line[-2] + 2 * line[-1] + 4 * line[0] + 2 * line[1] + line[2]);
    };
    scanRun.gaussSecond = [](std::array<int, 5>& gaussBuffer, int y) // 5x5 gauss vertical
    {
      return gaussBuffer[(y - 2) % 5] + 2 * gaussBuffer[(y - 1) % 5] + 4 * gaussBuffer[y % 5] +
             2 * gaussBuffer[(y + 1) % 5] + gaussBuffer[(y + 2) % 5];
    };
    scanRun.gradient = [](std::array<int, 5>& gaussBuffer, int y) // sobel-like gradient
    {
      return gaussBuffer[(y - 2) % 5] + 2 * gaussBuffer[(y - 1) % 5] - 2 * gaussBuffer[(y + 1) % 5] - gaussBuffer[(y + 2) % 5];
    };
    // Initialize the buffer of smoothed values.
    scanRun.lowerGaussBuffer = {}; // buffer for the lower grid point and for sobel scans
    scanRun.upperGaussBuffer = {}; // buffer for the upper grid point, centered around grid point -> gridX is at index 2
    scanRun.lowerGaussBuffer[0] = scanRun.gaussH(luminance + 2 * theECImage.grayscaled.width);
    scanRun.lowerGaussBuffer[1] = scanRun.gaussH(luminance + theECImage.grayscaled.width);
    scanRun.lowerGaussBuffer[2] = scanRun.gaussH(luminance);
    scanRun.lowerGaussBuffer[3] = scanRun.gaussH(luminance -= theECImage.grayscaled.width);
    scanRun.lowerGaussBuffer[4] = scanRun.gaussH(luminance -= theECImage.grayscaled.width);
    gridValue = scanRun.gaussSecond(scanRun.lowerGaussBuffer, 2);

    while(nextGridY > middleY && gridYIndex <= theScanGrid.lowResHorizontalLines.size())
    {
      scanRun.upperGaussBuffer[0] = scanRun.gaussH(&theECImage.grayscaled[nextGridY + 2][line.x]);
      scanRun.upperGaussBuffer[1] = scanRun.gaussH(&theECImage.grayscaled[nextGridY + 1][line.x]);
      scanRun.upperGaussBuffer[2] = scanRun.gaussH(&theECImage.grayscaled[nextGridY][line.x]);
      scanRun.upperGaussBuffer[3] = scanRun.gaussH(&theECImage.grayscaled[nextGridY - 1][line.x]);
      scanRun.upperGaussBuffer[4] = scanRun.gaussH(&theECImage.grayscaled[nextGridY - 2][line.x]);
      nextGridValue = scanRun.gaussSecond(scanRun.upperGaussBuffer, 2);
      if(gridValue - nextGridValue >= threshold)
      {
        findEdgeInSubLineVertical(regions, scanRun, gridY, nextGridY, true);
      }
      else if(gridValue - nextGridValue <= -threshold)
      {
        findEdgeInSubLineVertical(regions, scanRun, gridY, nextGridY, false);
      }
      // prepare next loop
      gridY = nextGridY;
      while(gridY < nextGridY + verticalGridScanMinStep)
      {
        ++gridYIndex;
        if(gridYIndex < theScanGrid.fullResY.size())
          nextGridY = theScanGrid.fullResY[gridYIndex];
        else
          break;
      }
      gridValue = nextGridValue;
      scanRun.lowerGaussBuffer = scanRun.upperGaussBuffer;
    }
    lowerY = scanRun.lowerScanEdgePosition;
  }
  // switch to 3x3 filter
  if(middleY > top)
  {
    auto gauss = [](const PixelTypes::GrayscaledPixel* line) // 3x3 gauss horizontal
    {
      return static_cast<int>(line[-1] + 2 * line[0] + line[1]);
    };
    auto gaussSecond = [](std::array<int, 3>& gaussBuffer, int y) // 3x3 gauss vertical
    {
      return gaussBuffer[(y - 1) % 3] + 2 * gaussBuffer[y % 3] + gaussBuffer[(y + 1) % 3];
    };
    auto gradient = [](std::array<int, 3>& gaussBuffer, int y)
    {
      return gaussBuffer[(y - 1) % 3] - gaussBuffer[(y + 1) % 3];
    };
    // Initialize the buffer of smoothed values.
    std::array<int, 3> gaussBuffer {};
    luminance = &theECImage.grayscaled[gridY][line.x];
    gaussBuffer[0] = gauss(luminance + theECImage.grayscaled.width);
    gaussBuffer[1] = gauss(luminance);
    int gaussBufferIndex = 2;
    int prevSobelMin = std::numeric_limits<int>::max();
    int prevSobelMax = std::numeric_limits<int>::min();
    unsigned int edgeYMin = theECImage.grayscaled.height;
    unsigned int edgeYMax = theECImage.grayscaled.height;
    // adjust values for switch from 5x5 to 3x3
    gridValue = gridValue * 4 / 25; // smoothing factor is reduced from 100 to 16
    threshold = static_cast<int>(16.f * edgeThreshold); // "16 *" because it relates to the smoothed values.
    for(int y = gridY; y > top; --y)
    {
      // This line is one above the current y.
      gaussBuffer[gaussBufferIndex % 3] = gauss(luminance -= theECImage.grayscaled.width);
      ++gaussBufferIndex;
      // This gradient is centered around the current y.
      int sobelL = gradient(gaussBuffer, gaussBufferIndex - 2);
      if(sobelL > prevSobelMax)
      {
        prevSobelMax = sobelL;
        edgeYMax = y;
      }
      if(sobelL < prevSobelMin)
      {
        prevSobelMin = sobelL;
        edgeYMin = y;
      }
      if(y == gridY)
      {
        gridValue = gaussSecond(gaussBuffer, gaussBufferIndex - 2);
      }
      else if(y == nextGridY)
      {
        nextGridValue = gaussSecond(gaussBuffer, gaussBufferIndex - 2);
        if(gridValue - nextGridValue >= threshold)
        {
          ASSERT(lowerY > edgeYMax);
          regions.emplace_back(edgeYMax, lowerY, getVerticalRepresentativeValue(theECImage.grayscaled, line.x, edgeYMax, lowerY),
                               getVerticalRepresentativeHueValue(theECImage.hued, line.x, edgeYMax, lowerY),
                               getVerticalRepresentativeValue(theECImage.saturated, line.x, edgeYMax, lowerY));
          lowerY = edgeYMax;
          MID_DOT("module:ScanLineRegionizer:verticalRegionSplit", line.x, edgeYMax, ColorRGBA::magenta, ColorRGBA::magenta);
          if(prelabelAsWhite && nextRegionWhite && prelabelWhiteCheck(regions.back()) &&
             prelabelWhiteNeighborCheck(regions.back(), regions[regions.size() - 2]))
            regions.back().color = ScanLineRegion::white;
          nextRegionWhite = false;
        }
        else if(gridValue - nextGridValue <= -threshold)
        {
          ASSERT(lowerY > edgeYMin);
          regions.emplace_back(edgeYMin, lowerY, getVerticalRepresentativeValue(theECImage.grayscaled, line.x, edgeYMin, lowerY),
                               getVerticalRepresentativeHueValue(theECImage.hued, line.x, edgeYMin, lowerY),
                               getVerticalRepresentativeValue(theECImage.saturated, line.x, edgeYMin, lowerY));
          lowerY = edgeYMin;
          MID_DOT("module:ScanLineRegionizer:verticalRegionSplit", line.x, edgeYMin, ColorRGBA::magenta, ColorRGBA::magenta);
          nextRegionWhite = true;
        }
        if(prelabelAsWhite && regions.size() >= 2 && regions[regions.size() - 2].color == ScanLineRegion::white)
        {
          if(!prelabelWhiteNeighborCheck(regions[regions.size() - 2], regions.back()))
            regions[regions.size() - 2].color = ScanLineRegion::unset;
        }
        gridY = nextGridY;
        while(gridY < nextGridY + verticalGridScanMinStep)
        {
          ++gridYIndex;
          if(gridYIndex < theScanGrid.fullResY.size())
            nextGridY = theScanGrid.fullResY[gridYIndex];
          else
            break;
        }
        gridValue = nextGridValue;
        prevSobelMax = std::numeric_limits<int>::min();
        prevSobelMin = std::numeric_limits<int>::max();
        edgeYMax = theECImage.grayscaled.height;
        edgeYMin = theECImage.grayscaled.height;
      }
    }
  }
  // add last region
  ASSERT(lowerY > static_cast<unsigned int>(top));
    regions.emplace_back(top, lowerY, getVerticalRepresentativeValue(theECImage.grayscaled, line.x, top, lowerY),
                         getVerticalRepresentativeHueValue(theECImage.hued, line.x, top, lowerY),
                         getVerticalRepresentativeValue(theECImage.saturated, line.x, top, lowerY));
}

template <int filterSize>
void ScanLineRegionizer::findEdgeInSubLineHorizontal(
    std::vector<InternalRegion>& regions,
    ScanRun<filterSize>& scanRun,
    unsigned int startPos,
    unsigned int stopPos,
    bool maxEdge) const
{
  unsigned int edgeXMax = startPos;
  int sobelMax = scanRun.gradient(scanRun.leftGaussBuffer, (filterSize - 1) / 2);
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[scanRun.scanLinePosition][startPos + 1];
  int gaussBufferIndex = 0;
  for(unsigned int x = startPos + 1; x < stopPos; ++x, ++luminance, ++gaussBufferIndex)
  {
    scanRun.leftGaussBuffer[gaussBufferIndex % filterSize] = scanRun.gaussV(luminance, theECImage.grayscaled.width);
    int sobelL = scanRun.gradient(scanRun.leftGaussBuffer, gaussBufferIndex + (filterSize + 1) / 2);
    if((maxEdge && sobelL > sobelMax) || (!maxEdge && sobelL < sobelMax))
    {
      edgeXMax = x;
      sobelMax = sobelL;
    }
  }
  // save region
  ASSERT(scanRun.leftScanEdgePosition < edgeXMax);
  regions.emplace_back(scanRun.leftScanEdgePosition, edgeXMax, getHorizontalRepresentativeValue(theECImage.grayscaled, scanRun.leftScanEdgePosition, edgeXMax, scanRun.scanLinePosition),
                       getHorizontalRepresentativeHueValue(theECImage.hued, scanRun.leftScanEdgePosition, edgeXMax, scanRun.scanLinePosition),
                       getHorizontalRepresentativeValue(theECImage.saturated, scanRun.leftScanEdgePosition, edgeXMax, scanRun.scanLinePosition));
  scanRun.leftScanEdgePosition = edgeXMax;
  MID_DOT("module:ScanLineRegionizer:horizontalRegionSplit", edgeXMax, scanRun.scanLinePosition, ColorRGBA::magenta, ColorRGBA::magenta);
}

template <int filterSize>
void ScanLineRegionizer::findEdgeInSubLineVertical(
    std::vector<InternalRegion>& regions,
    ScanRun<filterSize>& scanRun,
    unsigned int startPos,
    unsigned int stopPos,
    bool maxEdge) const
{
  unsigned int edgeYMax = startPos;
  int sobelMax = scanRun.gradient(scanRun.lowerGaussBuffer, (filterSize - 1) / 2);
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[startPos - 1][scanRun.scanLinePosition];
  int gaussBufferIndex = 0;
  for(int y = static_cast<int>(startPos) - 1; y > static_cast<int>(stopPos); --y, ++gaussBufferIndex, luminance -= theECImage.grayscaled.width)
  {
    scanRun.lowerGaussBuffer[gaussBufferIndex % filterSize] = scanRun.gaussH(luminance);
    int sobelL = scanRun.gradient(scanRun.lowerGaussBuffer, gaussBufferIndex + (filterSize + 1) / 2);
    if((maxEdge && sobelL > sobelMax) || (!maxEdge && sobelL < sobelMax))
    {
      edgeYMax = y;
      sobelMax = sobelL;
    }
  }
  // save region
  ASSERT(scanRun.lowerScanEdgePosition > edgeYMax);
  regions.emplace_back(edgeYMax, scanRun.lowerScanEdgePosition, getVerticalRepresentativeValue(theECImage.grayscaled, scanRun.scanLinePosition, edgeYMax, scanRun.lowerScanEdgePosition),
                       getVerticalRepresentativeHueValue(theECImage.hued, scanRun.scanLinePosition, edgeYMax, scanRun.lowerScanEdgePosition),
                       getVerticalRepresentativeValue(theECImage.saturated, scanRun.scanLinePosition, edgeYMax, scanRun.lowerScanEdgePosition));
  scanRun.lowerScanEdgePosition = edgeYMax;
  MID_DOT("module:ScanLineRegionizer:verticalRegionSplit", scanRun.scanLinePosition, edgeYMax, ColorRGBA::magenta, ColorRGBA::magenta);
}

void ScanLineRegionizer::uniteHorizontalFieldRegions(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions) const
{
  ASSERT(y.size() == regions.size());
  for(std::size_t lineIndex = 0; lineIndex < y.size(); ++lineIndex)
  {
    std::size_t nextLineIndex = lineIndex + 1;
    std::vector<InternalRegion>::iterator nextLineRegion;
    if(nextLineIndex < regions.size())
      nextLineRegion = regions[nextLineIndex].begin();

    for(std::size_t regionIndex = 0; regionIndex < regions[lineIndex].size(); ++regionIndex)
    {
      InternalRegion& region = regions[lineIndex][regionIndex];
      // try union with next region in the same line
      if(regionIndex + 1 < regions[lineIndex].size())
      {
        if(areSimilar(region, regions[lineIndex][regionIndex + 1]))
        {
          region.unite(regions[lineIndex][regionIndex + 1]);
          LINE("module:ScanLineRegionizer:horizontalRegionUnion", (region.range.from + region.range.to) / 2.f, y[lineIndex],
               (regions[lineIndex][regionIndex + 1].range.from + regions[lineIndex][regionIndex + 1].range.to) / 2.f, y[lineIndex],
               2, Drawings::PenStyle::solidPen, ColorRGBA::red);
        }
      }
      // try union with adjacent regions in next line
      if(nextLineIndex < regions.size() && !regions[nextLineIndex].empty())
      {
        bool entered = false;
        // regions in the horizontal scan lines are sorted from lower to higher pixel number
        while(nextLineRegion != regions[nextLineIndex].end() && nextLineRegion->range.from < region.range.to)
        {
          if(areSimilar(region, *nextLineRegion))
          {
            region.unite(*nextLineRegion);
            LINE("module:ScanLineRegionizer:horizontalRegionUnion", (region.range.from + region.range.to) / 2.f, y[lineIndex],
                 (nextLineRegion->range.from + nextLineRegion->range.to) / 2.f, y[nextLineIndex], 2, Drawings::PenStyle::solidPen, ColorRGBA::red);
          }
          ++nextLineRegion;
          entered = true;
        }
        // The last region on the next line adjacent to the current region on this line is also adjacent to the next region on this line
        if(entered)
          --nextLineRegion;
      }
    }
  }
}

void ScanLineRegionizer::uniteVerticalFieldRegions(const std::vector<unsigned short>& x, std::vector<std::vector<InternalRegion>>& regions) const
{
  ASSERT(x.size() == regions.size());
  for(std::size_t lineIndex = 0; lineIndex < regions.size(); ++lineIndex)
  {
    // build list of regions in the neighborhood of the line. Can span multiple scan lines due to ScanGrid layout
    if(regions[lineIndex].empty())
      continue;
    unsigned short lineRangeFrom = regions[lineIndex][regions[lineIndex].size() - 1].range.from;
    const unsigned short lineRangeTo = regions[lineIndex][0].range.to;
    ASSERT(lineRangeFrom < lineRangeTo);
    std::list<std::pair<InternalRegion*, unsigned short>> nextLinesRegions;
    for(std::size_t nextLineIndex = lineIndex + 1; nextLineIndex < regions.size() && nextLineIndex <= lineIndex + 4; ++nextLineIndex)
    {
      if(lineRangeFrom >= lineRangeTo)
        break;
      for(long nextLineRegionIndex = static_cast<long>(regions[nextLineIndex].size()) - 1; nextLineRegionIndex >= 0; --nextLineRegionIndex)
      {
        if(regions[nextLineIndex][nextLineRegionIndex].range.to <= lineRangeFrom)
          continue;
        else if(regions[nextLineIndex][nextLineRegionIndex].range.from >= lineRangeTo)
          break;
        else
        {
          nextLinesRegions.emplace_back(&regions[nextLineIndex][nextLineRegionIndex], x[nextLineIndex]);
          lineRangeFrom = regions[nextLineIndex][nextLineRegionIndex].range.to;
        }
      }
    }
    std::list<std::pair<InternalRegion*, unsigned short>>::iterator nextLineRegion;
    if(!nextLinesRegions.empty())
      nextLineRegion = nextLinesRegions.begin();

    for(long regionIndex = static_cast<long>(regions[lineIndex].size()) - 1; regionIndex >= 0 ; --regionIndex)
    {
      InternalRegion& region = regions[lineIndex][regionIndex];
      // try union with next region in the same line
      if(regionIndex > 0)
      {
        if(areSimilar(region, regions[lineIndex][regionIndex - 1]))
        {
          region.unite(regions[lineIndex][regionIndex - 1]);
          LINE("module:ScanLineRegionizer:verticalRegionUnion", x[lineIndex], (region.range.from + region.range.to) / 2.f,
               x[lineIndex], (regions[lineIndex][regionIndex - 1].range.from + regions[lineIndex][regionIndex - 1].range.to) / 2.f,
               2, Drawings::PenStyle::solidPen, ColorRGBA::red);
        }
      }
      // try union with adjacent regions in next lines
      if(!nextLinesRegions.empty())
      {
        // regions in the vertical scan lines are sorted from higher to lower pixel number
        bool loopEntered = false;
        while(nextLineRegion != nextLinesRegions.end() && nextLineRegion->first->range.from < region.range.to)
        {
          if(areSimilar(region, *(nextLineRegion->first)))
          {
            region.unite(*(nextLineRegion->first));
            LINE("module:ScanLineRegionizer:verticalRegionUnion", x[lineIndex], (region.range.from + region.range.to) / 2.f,
                 nextLineRegion->second, (nextLineRegion->first->range.from + nextLineRegion->first->range.to) / 2.f, 2, Drawings::PenStyle::solidPen, ColorRGBA::red);
          }
          loopEntered = true;
          ++nextLineRegion;
        }
        if(loopEntered)
          --nextLineRegion;
      }
    }
  }
}

bool ScanLineRegionizer::areSimilar(InternalRegion& a, InternalRegion& b) const
{
  const InternalRegion* const aParent = a.findSet();
  const InternalRegion* const bParent = b.findSet();
  // test for white because of white prelabeling -> prelabeled regions should not be changed
  if(std::abs(static_cast<short>(aParent->y) - static_cast<short>(bParent->y)) < luminanceSimilarityThreshold &&
     std::abs(static_cast<short>(aParent->h) - static_cast<short>(bParent->h)) < hueSimilarityThreshold &&
     std::abs(static_cast<short>(aParent->s) - static_cast<short>(bParent->s)) < saturationSimilarityThreshold &&
     aParent->color != ScanLineRegion::white && bParent->color != ScanLineRegion::white)
    return true;
  return false;
}

void ScanLineRegionizer::classifyFieldRegions(const std::vector<unsigned short>& xy, std::vector<std::vector<InternalRegion>>& regions, bool horizontal)
{
  unsigned char minHue = 255;
  unsigned char maxHue = 0;
  unsigned char minSaturation = initialMinSaturation;
  // luminance is only important for images with low saturated field
  unsigned char maxLuminance = baseLuminance;
  int dataPoints = 0;
  for(auto& line : regions)
  {
    for(auto& region : line)
    {
      InternalRegion* representative = region.findSet();
      // ASSERTs that all Regions start with color == ScanLineRegion::unset !
      if(representative->color == ScanLineRegion::unset)
      {
        if(regionIsField(representative))
        {
          representative->color = ScanLineRegion::field;
          ++dataPoints;
          if(representative->h < minHue)
            minHue = representative->h;
          if(representative->h > maxHue)
            maxHue = representative->h;
          if(representative->s < minSaturation)
            minSaturation = representative->s;
          if(representative->y > maxLuminance)
            maxLuminance = representative->y;
        }
        else
          representative->color = ScanLineRegion::none;
      }
      region.color = representative->color;
    }
  }
  if(dataPoints > 0)
  {
    // expand min/max range
    // the values from the representative regions are averages, but we need sensible min/max-values
    // higher min/max range expansion if we have few data points
    const unsigned char hueRangeExpansion = 8 + static_cast<unsigned char>(2.f * static_cast<float>(hueSimilarityThreshold) / static_cast<float>(dataPoints + 1));
    const unsigned char satRangeExpansion = 5 + static_cast<unsigned char>(2.f * static_cast<float>(saturationSimilarityThreshold) / static_cast<float>(dataPoints + 1));
    const unsigned char lumRangeExpansion = 5 + static_cast<unsigned char>(
                                              0.5f * static_cast<float>(baseLuminance) + 1.5f * static_cast<float>(luminanceSimilarityThreshold) / static_cast<float>(dataPoints + 1));
    estimatedFieldColor.minHue = hueRangeExpansion >= minHue ? 0 : minHue - hueRangeExpansion;
    estimatedFieldColor.maxHue = hueRangeExpansion >= std::numeric_limits<unsigned char>::max() - maxHue ?
                                 std::numeric_limits<unsigned char>::max() : maxHue + hueRangeExpansion;
    estimatedFieldColor.minSaturation = satRangeExpansion >= minSaturation ? 0 : minSaturation - satRangeExpansion;
    estimatedFieldColor.maxLuminance = lumRangeExpansion >= std::numeric_limits<unsigned char>::max() - maxLuminance ?
                                       std::numeric_limits<unsigned char>::max() : maxLuminance + lumRangeExpansion;
    estimatedFieldColor.lastSet = theFrameInfo.time;
  }
  // label smaller field regions
  if(isEstimatedFieldColorValid())
  {
    if(horizontal)
    {
      classifyFieldHorizontal(xy, regions);
      drawEstimatedFieldColorRangeHorizontal(dataPoints);
    }
    else
    {
      classifyFieldVertical(regions);
      drawEstimatedFieldColorRangeVertical(dataPoints);
    }
  }
}

void ScanLineRegionizer::classifyFieldHorizontal(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions) const
{
  ASSERT(y.size() == regions.size());
  for(size_t lineIndex = 0; lineIndex < regions.size(); ++lineIndex)
  {
    auto& line = regions[lineIndex];
    for(auto& region : line)
    {
      classifyFieldSingleRegion(region,  y[lineIndex], true);
    }
  }
}

void ScanLineRegionizer::classifyFieldVertical(std::vector<std::vector<InternalRegion>>& regions) const
{
  for(auto& line : regions)
  {
    for(auto& region : line)
    {
      classifyFieldSingleRegion(region, 0, false);
    }
  }
}

void ScanLineRegionizer::classifyFieldSingleRegion(ScanLineRegionizer::InternalRegion& region, float regionHeight, bool horizontal) const
{
  ASSERT(region.color != ScanLineRegion::unset);
  if(region.color == ScanLineRegion::none)
  {
    if(region.findSet()->color == ScanLineRegion::field)
    {
      region.color = ScanLineRegion::field;
      return;
    }

    if(!horizontal)
      regionHeight = static_cast<float>(region.range.from + region.range.to) / 2.f;

    unsigned char minSaturation = fieldClassificationSaturationThreshold(region, regionHeight);
    char hueRangeExpansion = 0;
    if(theCameraInfo.camera == CameraInfo::upper)
      hueRangeExpansion = static_cast<char>(0.5f * static_cast<float>(hueSimilarityThreshold) - (regionHeight - static_cast<float>(theScanGrid.fieldLimit)) /
                                                                                                static_cast<float>(theCameraInfo.height -
                                                                                                                   theScanGrid.fieldLimit) * 0.5f *
                                                                                                static_cast<float>(hueSimilarityThreshold));

    if(region.h >= estimatedFieldColor.minHue - hueRangeExpansion &&
       region.h <= estimatedFieldColor.maxHue + hueRangeExpansion &&
       region.s >= minSaturation && region.y <= estimatedFieldColor.maxLuminance)
      region.color = ScanLineRegion::field;
    else
      region.color = ScanLineRegion::none;
  }
}

unsigned char ScanLineRegionizer::fieldClassificationSaturationThreshold(const InternalRegion& region, float regionHeight) const
{
  // saturation of very dark field pixels is often 0
  if(region.y > 2)
  {
    if(theCameraInfo.camera == CameraInfo::upper)
    {
      // reduce saturation threshold for further away regions
      float scalingFactor = ((regionHeight - static_cast<float>(theScanGrid.fieldLimit)) /
                                   static_cast<float>(theCameraInfo.height - theScanGrid.fieldLimit) + 1.f) / 1.8f;
      return static_cast<unsigned char>(static_cast<float>(estimatedFieldColor.minSaturation) * scalingFactor);
    }
    return estimatedFieldColor.minSaturation;
  }
  return 0;
}

bool ScanLineRegionizer::regionIsField(const InternalRegion* const region) const
{
  if((theCameraInfo.camera == CameraInfo::lower && region->regionSize > lowerMinRegionSize) ||
     (theCameraInfo.camera == CameraInfo::upper && region->regionSize > upperMinRegionSize))
  {
    if(region->y <= theRelativeFieldColorsParameters.maxFieldLuminance &&
       region->s >= theRelativeFieldColorsParameters.minFieldSaturation &&
       theRelativeFieldColorsParameters.fieldHue.isInside(region->h))
      return true;
  }
  return false;
}

void ScanLineRegionizer::classifyWhiteRegionsWithThreshold(std::vector<std::vector<InternalRegion>>& regions) const
{
  unsigned char minWhiteLuminance = static_cast<unsigned char>(
                                      std::min(std::min(static_cast<int>(estimatedFieldColor.maxLuminance), static_cast<int>(theRelativeFieldColorsParameters.maxFieldLuminance)),
                                               static_cast<int>(baseLuminance) + luminanceSimilarityThreshold));
  unsigned char maxWhiteSaturation = static_cast<unsigned char>(
                                       std::min(static_cast<int>(theRelativeFieldColorsParameters.maxWhiteSaturation),
                                                std::max(static_cast<int>(static_cast<float>(theRelativeFieldColorsParameters.minFieldSaturation) + (static_cast<float>(estimatedFieldColor.minSaturation) -
                                                         static_cast<float>(theRelativeFieldColorsParameters.minFieldSaturation)) * 0.7f),
                                                         static_cast<int>(static_cast<float>(baseSaturation) * 0.5f) - saturationSimilarityThreshold)));
  COMPLEX_DRAWING("module:ScanLineRegionizer:whiteRange")
  {
    int fontSize = 9;
    if(theCameraInfo.camera == CameraInfo::upper)
      fontSize = 14;
    const int xPos = 10 + theCameraInfo.width * 2 / 3;
    int yPos = fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:whiteRange", xPos, yPos, fontSize, ColorRGBA::orange, "MinLum: " << minWhiteLuminance);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:whiteRange", xPos, yPos, fontSize, ColorRGBA::orange, "MaxSat: " << maxWhiteSaturation);
  }
  for(auto& line : regions)
  {
    for(auto& region : line)
    {
      if(region.color != ScanLineRegion::field && region.y > minWhiteLuminance && region.s < maxWhiteSaturation)
        region.color = ScanLineRegion::white;
    }
  }
}

bool ScanLineRegionizer::prelabelWhiteCheck(const InternalRegion& checkedRegion) const
{
  return checkedRegion.regionSize <= maxPrelabelRegionSize &&
         checkedRegion.y >= baseLuminance && checkedRegion.y >= baseLuminance + luminanceSimilarityThreshold / 2 &&
         checkedRegion.s <= baseSaturation - saturationSimilarityThreshold / 2;
}

bool ScanLineRegionizer::prelabelWhiteNeighborCheck(const InternalRegion& checkedRegion, const InternalRegion& neighborRegion) const
{
  int neighborRegionSize = neighborRegion.range.to - neighborRegion.range.from;
  float thresholdModifier = neighborRegionSize >= 10 ? 1 : static_cast<float>(neighborRegionSize) / 10.f;
  return static_cast<int>(checkedRegion.y) > static_cast<int>(neighborRegion.y) &&
         static_cast<int>(checkedRegion.s) < static_cast<int>(neighborRegion.s) &&
         static_cast<int>(checkedRegion.y) + static_cast<int>(neighborRegion.s) >=
         static_cast<int>(neighborRegion.y) + static_cast<int>(checkedRegion.s) +
         static_cast<int>(thresholdModifier * static_cast<float>(luminanceSimilarityThreshold + saturationSimilarityThreshold));
}

void ScanLineRegionizer::stitchUpHoles(std::vector<std::vector<InternalRegion>>& regions, bool horizontal) const
{
  ScanLineRegion::Color lastColor = ScanLineRegion::none;
  ScanLineRegion::Color currentColor = ScanLineRegion::none;
  size_t lastColorIndex;
  int currentRegionSize;
  for(auto& line : regions)
  {
    if(line.size() < 3)
      continue;
    lastColor = line[0].color;
    currentColor = line[1].color;
    lastColorIndex = 0;
    currentRegionSize = line[1].range.to - line[1].range.from;
    for(size_t regionIndex = 2; regionIndex < line.size(); ++regionIndex)
    {
      if(currentColor == ScanLineRegion::none && currentRegionSize <= maxRegionSizeForStitching)
      {
        if(line[regionIndex].color == ScanLineRegion::none && regionIndex + 1 < line.size() &&
           currentRegionSize + line[regionIndex].range.to - line[regionIndex].range.from <= maxRegionSizeForStitching)
        {
          // Successive "none" regions that can be changed together
          currentRegionSize += line[regionIndex].range.to - line[regionIndex].range.from;
          continue;
        }
        if(lastColor == ScanLineRegion::field && line[regionIndex].color == ScanLineRegion::field)
        {
          for(size_t changeIndex = lastColorIndex + 1; changeIndex < regionIndex; ++changeIndex)
            line[changeIndex].color = ScanLineRegion::field;
        }
        else if((lastColor == ScanLineRegion::field && line[regionIndex].color == ScanLineRegion::white) ||
                (lastColor == ScanLineRegion::white && line[regionIndex].color == ScanLineRegion::field))
        {
          if(horizontal)
          {
            unsigned short stitchPos = (line[lastColorIndex].range.to + line[regionIndex].range.from + 1) / 2;
            line[lastColorIndex].range.to = stitchPos;
            line[regionIndex].range.from = stitchPos;
          }
          else
          {
            // vertical
            unsigned short stitchPos = (line[lastColorIndex].range.from + line[regionIndex].range.to) / 2;
            line[lastColorIndex].range.from = stitchPos;
            line[regionIndex].range.to = stitchPos;
          }
          line.erase(line.cbegin() + static_cast<long>(lastColorIndex) + 1, line.cbegin() + static_cast<long>(regionIndex));
          regionIndex = lastColorIndex + 1;
          currentColor = lastColor;
        }
      }
      lastColor = currentColor;
      currentColor = line[regionIndex].color;
      lastColorIndex = regionIndex - 1;
      currentRegionSize = line[regionIndex].range.to - line[regionIndex].range.from;
    }
  }
}

void ScanLineRegionizer::emplaceInScanLineRegionsHorizontal(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal,
                                        const std::vector<unsigned short>& yPerScanLine,
                                        const std::vector<std::vector<InternalRegion>>& regionsPerScanLine)
{
  ASSERT(yPerScanLine.size() == regionsPerScanLine.size());
  const std::size_t numOfScanLines = yPerScanLine.size();
  colorScanLineRegionsHorizontal.scanLines.reserve(numOfScanLines);
  for(std::size_t i = 0; i < numOfScanLines; ++i)
  {
    colorScanLineRegionsHorizontal.scanLines.emplace_back(yPerScanLine[i]);
    const std::vector<InternalRegion>& regions = regionsPerScanLine[i];
    std::vector<ScanLineRegion>& newRegions = colorScanLineRegionsHorizontal.scanLines[i].regions;
    for(std::size_t j = 0; j < regions.size(); ++j)
    {
      // Merge adjacent regions of the same color.
      if(!j || newRegions.back().color != regions[j].color)
        newRegions.emplace_back(regions[j].range.from, regions[j].range.to, regions[j].color);
      else
        newRegions.back().range.to = regions[j].range.to;
    }
  }
}

void ScanLineRegionizer::emplaceInScanLineRegionsVertical(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped,
                                      const std::vector<unsigned short>& xPerScanLine,
                                      const std::vector<std::vector<InternalRegion>>& regionsPerScanLine)
{
  ASSERT(xPerScanLine.size() == regionsPerScanLine.size());
  std::size_t numOfScanLines = xPerScanLine.size();
  colorScanLineRegionsVerticalClipped.scanLines.reserve(numOfScanLines);
  for(std::size_t i = 0; i < numOfScanLines; ++i)
  {
    colorScanLineRegionsVerticalClipped.scanLines.emplace_back(xPerScanLine[i]);
    const std::vector<InternalRegion>& regions = regionsPerScanLine[i];
    std::vector<ScanLineRegion>& newRegions = colorScanLineRegionsVerticalClipped.scanLines[i].regions;
    for(std::size_t j = 0; j < regions.size(); ++j)
    {
      // Merge adjacent regions of the same color.
      if(!j || newRegions.back().color != regions[j].color)
        newRegions.emplace_back(regions[j].range.from, regions[j].range.to, regions[j].color);
      else
        newRegions.back().range.from = regions[j].range.from;
    }
  }
}

bool ScanLineRegionizer::isEstimatedFieldColorValid() const
{
  return theFrameInfo.getTimeSince(estimatedFieldColor.lastSet) <= estimatedFieldColorInvalidationTime;
}

void ScanLineRegionizer::drawEstimatedFieldColorRangeHorizontal([[maybe_unused]] int dataPoints) const
{
  COMPLEX_DRAWING("module:ScanLineRegionizer:fieldColorRangeHorizontal")
  {
    int fontSize = 9;
    if(theCameraInfo.camera == CameraInfo::upper)
      fontSize = 14;
    const int xPos = 10 + theCameraInfo.width / 3;
    int yPos = fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeHorizontal", xPos, yPos, fontSize, ColorRGBA::orange, "Horizontal");
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeHorizontal", xPos, yPos, fontSize, ColorRGBA::orange, "MinHue: " << estimatedFieldColor.minHue);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeHorizontal", xPos, yPos, fontSize, ColorRGBA::orange, "MaxHue: " << estimatedFieldColor.maxHue);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeHorizontal", xPos, yPos, fontSize, ColorRGBA::orange, "MinSat: " << estimatedFieldColor.minSaturation);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeHorizontal", xPos, yPos, fontSize, ColorRGBA::orange, "MaxLum: " << estimatedFieldColor.maxLuminance);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeHorizontal", xPos, yPos, fontSize, ColorRGBA::orange, "DataPoints: " << dataPoints);
  }
}

void ScanLineRegionizer::drawEstimatedFieldColorRangeVertical([[maybe_unused]] int dataPoints) const
{
  COMPLEX_DRAWING("module:ScanLineRegionizer:fieldColorRangeVertical")
  {
    int fontSize = 9;
    if(theCameraInfo.camera == CameraInfo::upper)
      fontSize = 14;
    int yPos = fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeVertical", 10, yPos, fontSize, ColorRGBA::orange, "Vertical");
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeVertical", 10, yPos, fontSize, ColorRGBA::orange, "MinHue: " << estimatedFieldColor.minHue);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeVertical", 10, yPos, fontSize, ColorRGBA::orange, "MaxHue: " << estimatedFieldColor.maxHue);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeVertical", 10, yPos, fontSize, ColorRGBA::orange, "MinSat: " << estimatedFieldColor.minSaturation);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeVertical", 10, yPos, fontSize, ColorRGBA::orange, "MaxLum: " << estimatedFieldColor.maxLuminance);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRangeVertical", 10, yPos, fontSize, ColorRGBA::orange, "DataPoints: " << dataPoints);
  }
}

PixelTypes::GrayscaledPixel ScanLineRegionizer::getHorizontalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image,
                                                                                 const unsigned int from, const unsigned int to, const unsigned int y)
{
  ASSERT(to > from);
  const unsigned int length = to - from;
  unsigned int sum = 0;
  if(length <= 6)
  {
    // take every pixel from very small regions
    for(unsigned int x = from; x < to; ++x)
      sum += image[y][x];
    return static_cast<unsigned char>(sum / length);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle-sized regions
    for(unsigned int x = from + 1; x < to - 1; x += 2) // don't start or stop in the edge of the region
      sum += image[y][x];
    return static_cast<unsigned char>(sum / ((length - 1) / 2));
  }
  else
  {
    // take every fourth pixel from bigger regions
    for(unsigned int x = from + 2; x < to - 1; x += 4) // don't start or stop in the edge of the region
      sum += image[y][x];
    // this relies on (length / 4) being floored; expanding to 4*sum/length will lead to false results
    return static_cast<unsigned char>(sum / (length / 4));
  }
}

PixelTypes::HuePixel ScanLineRegionizer::getHorizontalRepresentativeHueValue(const Image<PixelTypes::HuePixel>& image,
                                                                             unsigned int from, unsigned int to, unsigned int y)
{
  ASSERT(to > from);
  const unsigned int length = to - from;
  float hueValue = 0;
  int currentStep = 1;
  if(length <= 6)
  {
    // take every pixel from very small regions
    for(unsigned int x = from; x < to; ++x, ++currentStep)
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle-sized regions
    for(unsigned int x = from + 1; x < to - 1; x += 2, ++currentStep) // don't start or stop in the edge of the region
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
  }
  else
  {
    // take every fourth pixel from bigger regions
    for(unsigned int x = from + 2; x < to - 1; x += 4, ++currentStep) // don't start or stop in the edge of the region
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
  }
  return static_cast<unsigned char>(hueValue);
}

PixelTypes::GrayscaledPixel ScanLineRegionizer::getVerticalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image,
                                                                               const unsigned int x, const unsigned int from, const unsigned int to)
{
  ASSERT(to > from);
  const int length = static_cast<int>(to - from);
  unsigned int sum = 0;
  if(length <= 6)
  {
    // take every pixel from very small regions
    for(unsigned int y = from; y < to; ++y)
      sum += image[y][x];
    return static_cast<unsigned char>(sum / length);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle-sized regions
    for(unsigned int y = from + 1; y < to - 1; y += 2) // don't start or stop in the edge of the region
      sum += image[y][x];
    return static_cast<unsigned char>(sum / ((length - 1) / 2));
  }
  else
  {
    // take every forth pixel from sufficiently large regions
    for(unsigned int y = from + 2; y < to - 1; y += 4) // don't start or stop in the edge of the region
      sum += image[y][x];
    // this relies on (length / 4) being floored, so please don't change to 4*sum/length as that will lead to false results
    return static_cast<unsigned char>(sum / (length / 4));
  }
}

PixelTypes::HuePixel ScanLineRegionizer::getVerticalRepresentativeHueValue(const Image<PixelTypes::HuePixel>& image,
                                                                           const unsigned int x, const unsigned int from, const unsigned int to)
{
  ASSERT(to > from);
  const int length = static_cast<int>(to - from);
  float hueValue = 0;
  int currentStep = 1;
  if(length <= 6)
  {
    // take every pixel from very small regions
    for(unsigned int y = from; y < to; ++y, ++currentStep)
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle-sized regions
    for(unsigned int y = from + 1; y < to - 1; y += 2, ++currentStep) // don't start or stop in the edge of the region
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
  }
  else
  {
    // take every fourth pixel from sufficiently large regions
    for(unsigned int y = from + 2; y < to - 1; y += 4, ++currentStep) // don't start or stop in the edge of the region
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
  }
  return static_cast<unsigned char>(hueValue);
}

void ScanLineRegionizer::approximateBaseLuminance()
{
  baseLuminance = static_cast<PixelTypes::GrayscaledPixel>(baseLuminanceReduction * theRelativeFieldColors.averageLuminanceF);
}

void ScanLineRegionizer::approximateBaseSaturation()
{
  baseSaturation = theRelativeFieldColors.averageSaturation;
}

float ScanLineRegionizer::hueAverage(float hueValue, const float hueAddition, const int dataPoints)
{
  const float hueDiff = hueAddition - hueValue;
  hueValue += hueDiff / static_cast<float>(dataPoints);
  if(hueValue < 0.f)
    hueValue += 256.f;
  if(hueValue >= 256.f)
    hueValue -= 256.f;
  return hueValue;
}
