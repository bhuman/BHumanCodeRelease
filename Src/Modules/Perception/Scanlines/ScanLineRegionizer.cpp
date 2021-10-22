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
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"

#include <functional>
#include <list>
#include <vector>

MAKE_MODULE(ScanLineRegionizer, perception);

using PixelTypes::Color;

void ScanLineRegionizer::update(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal)
{
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:horizontalRegionSplit", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:horizontalRegionUnion", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:fieldColorRange", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:whiteRange", "drawingOnImage");

  colorScanLineRegionsHorizontal.scanLines.clear();

  if(theScanGrid.lines.empty() || theScanGrid.y.empty() || !theFieldBoundary.isValid)
    return;

  // 0. Preprocess
  approximateBaseLuminance();
  approximateBaseSaturation();

  // 1. Detect edges and create temporary regions in between including a representative YHS triple.
  std::size_t numOfScanLines = 0;
  std::vector<unsigned short> yPerScanLine;
  std::vector<std::vector<InternalRegion>> regionsPerScanLine;
  Vector2f pointInImage;
  const int middle = theCameraInfo.camera == CameraInfo::lower ? 0 :
                     Transformation::robotWithCameraRotationToImage(additionalSmoothingPoint, theCameraMatrix, theCameraInfo, pointInImage) ?
                     static_cast<const int>(pointInImage.y()) : theCameraInfo.height - 1;
  LINE("module:ScanLineRegionizer:horizontalRegionSplit", 0, middle, theECImage.grayscaled.width, middle, 3, Drawings::PenStyle::dottedPen, ColorRGBA(80, 6, 80));

  const int top = std::max(theFieldBoundary.getBoundaryTopmostY(theCameraInfo.width), theScanGrid.fieldLimit);
  int usedY = static_cast<int>(theECImage.grayscaled.height) + minHorizontalScanLineDistance;
  for(int y : theScanGrid.y)
  {
    if(usedY - y < minHorizontalScanLineDistance)
      usedY -= minHorizontalScanLineDistance;
    else
      usedY = y;
    if(usedY <= top)
      break;

    const unsigned int bodyLeft = theBodyContour.getRightEdge(usedY, theCameraInfo.width);
    const unsigned int bodyRight = theBodyContour.getLeftEdge(usedY, theCameraInfo.width);
    ASSERT(bodyLeft >= 0 && bodyLeft < static_cast<unsigned int>(theCameraInfo.width) &&
           bodyRight > 0 && bodyRight <= static_cast<unsigned int>(theCameraInfo.width));
    unsigned int boundaryLeft = horizontalScanStart(static_cast<int>(bodyLeft), usedY);
    unsigned int boundaryRight = horizontalScanStop(static_cast<int>(bodyRight), usedY);
    if(boundaryLeft >= static_cast<unsigned int>(theCameraInfo.width)) // may happen due to rounding
      boundaryLeft = 0;
    if(boundaryRight <= 0)
      boundaryRight = theCameraInfo.width;
    const unsigned int leftmostX = std::max(bodyLeft, boundaryLeft);
    const unsigned int rightmostX = std::min(bodyRight, boundaryRight);
    if(leftmostX >= rightmostX)
      continue;
    ++numOfScanLines;
    yPerScanLine.emplace_back(usedY);
    regionsPerScanLine.emplace_back(std::vector<InternalRegion>());

    if(theCameraInfo.camera == CameraInfo::lower || middle < usedY)
    {
      scanHorizontalAdditionalSmoothing(usedY, regionsPerScanLine.back(), leftmostX, rightmostX);
    }
    else
    {
      scanHorizontal(usedY, regionsPerScanLine.back(), leftmostX, rightmostX);
    }
  }

  // 2. Classify field regions.
  uniteHorizontalFieldRegions(yPerScanLine, regionsPerScanLine);
  classifyFieldRegions(yPerScanLine, regionsPerScanLine, true);

  // 3. Classify white regions.
  classifyWhiteRegionsWithThreshold(regionsPerScanLine);

  // 4. Fill small gaps between field and white regions
  stitchUpHoles(regionsPerScanLine, true);

  // 5. Convert to external representation.
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

void ScanLineRegionizer::update(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped)
{
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:verticalRegionSplit", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:verticalRegionUnion", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:fieldColorRange", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanLineRegionizer:whiteRange", "drawingOnImage");

  colorScanLineRegionsVerticalClipped.scanLines.clear();

  if(theScanGrid.lines.empty() || theScanGrid.y.empty() || !theFieldBoundary.isValid)
    return;

  colorScanLineRegionsVerticalClipped.lowResStart = theScanGrid.lowResStart;
  colorScanLineRegionsVerticalClipped.lowResStep = theScanGrid.lowResStep;

  // 0. Preprocess
  approximateBaseLuminance();
  approximateBaseSaturation();

  // 1. Detect edges and create temporary regions in between including a representative YHS triple.
  const std::size_t numOfScanLines = theScanGrid.lines.size();
  std::vector<unsigned short> xPerScanLine(numOfScanLines);
  std::vector<std::vector<InternalRegion>> regionsPerScanLine(numOfScanLines);
  Vector2f pointInImage;
  const int middle = theCameraInfo.camera == CameraInfo::lower ? 0 :
                     Transformation::robotWithCameraRotationToImage(additionalSmoothingPoint, theCameraMatrix, theCameraInfo, pointInImage) ?
                     static_cast<const int>(pointInImage.y()) : theCameraInfo.height - 1;
  LINE("module:ScanLineRegionizer:verticalRegionSplit", 0, middle, theECImage.grayscaled.width, middle, 3, Drawings::PenStyle::dottedPen, ColorRGBA(80, 6, 80));
  for(std::size_t i = 0; i < theScanGrid.lines.size(); ++i)
  {
    xPerScanLine[i] = static_cast<unsigned short>(theScanGrid.lines[i].x);
    const int top = std::max(theFieldBoundary.getBoundaryY(theScanGrid.lines[i].x), theScanGrid.fieldLimit) + 1;
    scanVertical(theScanGrid.lines[i], middle, top, regionsPerScanLine[i]);
  }

  // 2. Classify field regions.
  uniteVerticalFieldRegions(xPerScanLine, regionsPerScanLine);
  classifyFieldRegions(xPerScanLine, regionsPerScanLine, false);

  // 3. Classify white regions.
  classifyWhiteRegionsWithThreshold(regionsPerScanLine);

  // 4. Fill small gaps between field and white regions
  stitchUpHoles(regionsPerScanLine, false);

  // 5. Convert to external representation.
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

void ScanLineRegionizer::scanHorizontal(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const
{
  if(y < 1 || y >= theECImage.grayscaled.height - 1)
    return;
  // initialize variables
  const int scanY = static_cast<int>(y);
  unsigned int leftX = leftmostX;
  bool nextRegionWhite = false;
  const int thresholdAdaption = 16;
  int threshold = thresholdAdaption * static_cast<int>(edgeThreshold);
  // define filter
  auto gauss = [](const PixelTypes::GrayscaledPixel* line, const unsigned int imageWidth) // sobel smoothing
  {
    return static_cast<int>(line[-static_cast<int>(imageWidth)] + 2 * line[0] + line[imageWidth]);
  };
  auto gaussSecond = [](std::array<int, 3>& gaussBuffer, int x)
  {
    return gaussBuffer[(x - 1) % 3] + 2 * gaussBuffer[x % 3] + gaussBuffer[(x + 1) % 3];
  };
  auto gradient = [](std::array<int, 3>& gaussBuffer, int x)
  {
    return gaussBuffer[(x - 1) % 3] - gaussBuffer[(x + 1) % 3];
  };

  // Initialize the buffer of smoothed values.
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[scanY][leftX];
  std::array<int, 3> leftGauss{};  // buffer for the left grid point and for sobel scans
  std::array<int, 3> rightGauss{}; // buffer for the right grid point, centered around grid point -> gridX is at index 1
  leftGauss[0] = gauss(luminance, theECImage.grayscaled.width);
  leftGauss[1] = gauss(++luminance, theECImage.grayscaled.width);
  leftGauss[2] = gauss(++luminance, theECImage.grayscaled.width);
  int gaussBufferIndex = 0;
  // grid stuff
  unsigned int gridX = leftX + 1;
  int gridValue = gaussSecond(leftGauss, 1);
  int nextGridValue;
  size_t gridLineIndex = theScanGrid.lowResStart;
  unsigned int nextGridX = theScanGrid.lines[gridLineIndex].x;
  while(nextGridX <= gridX && nextGridX < rightmostX && ++gridLineIndex < theScanGrid.lines.size())
  {
    nextGridX = theScanGrid.lines[gridLineIndex].x;
  }
  if(nextGridX <= gridX && nextGridX < rightmostX)
    nextGridX = rightmostX; // skip the loop, make only a single region

  // 'gridLineIndex <= theScanGrid.lines.size()' so that the image's right edge becomes the last grid point
  while(gridLineIndex <= theScanGrid.lines.size() && nextGridX < rightmostX)
  {
    bool regionAdded = false;
    rightGauss[0] = gauss(&theECImage.grayscaled[scanY][nextGridX - 1], theECImage.grayscaled.width);
    rightGauss[1] = gauss(&theECImage.grayscaled[scanY][nextGridX], theECImage.grayscaled.width);
    rightGauss[2] = gauss(&theECImage.grayscaled[scanY][nextGridX + 1], theECImage.grayscaled.width);
    nextGridValue = gaussSecond(rightGauss, 1);
    if(gridValue - nextGridValue >= threshold)
    {
      // find exact edge position
      unsigned int edgeXMax = gridX;
      int sobelMax = gradient(leftGauss, 1);
      luminance = &theECImage.grayscaled[scanY][gridX + 1];
      gaussBufferIndex = 0;
      for(unsigned int x = gridX + 1; x < nextGridX; ++x, ++luminance, ++gaussBufferIndex)
      {
        leftGauss[gaussBufferIndex % 3] = gauss(luminance, theECImage.grayscaled.width);
        int sobelL = gradient(leftGauss, gaussBufferIndex + 2);
        if(sobelL > sobelMax)
        {
          edgeXMax = x;
          sobelMax = sobelL;
        }
      }
      // save region
      ASSERT(leftX < edgeXMax);
      regions.emplace_back(leftX, edgeXMax, getHorizontalRepresentativeValue(theECImage.grayscaled, leftX, edgeXMax, y),
                           getHorizontalRepresentativeHueValue(theECImage.hued, leftX, edgeXMax, y),
                           getHorizontalRepresentativeValue(theECImage.saturated, leftX, edgeXMax, y));
      leftX = edgeXMax;
      regionAdded = true;
      MID_DOT("module:ScanLineRegionizer:horizontalRegionSplit", edgeXMax, y, ColorRGBA::magenta, ColorRGBA::magenta);
      if(prelabelAsWhite && nextRegionWhite && prelabelWhiteCheck(regions.back()) &&
         prelabelWhiteNeighborCheck(regions.back(), regions[regions.size() - 2]))
      {
        regions.back().color = Color::white;
      }
      nextRegionWhite = false;
    }
    else if(gridValue - nextGridValue <= -threshold)
    {
      // find exact edge position
      unsigned int edgeXMin = gridX;
      int sobelMin = gradient(leftGauss, 1);
      luminance = &theECImage.grayscaled[scanY][gridX + 1];
      gaussBufferIndex = 0;
      for(unsigned int x = gridX; x < nextGridX; ++x, ++luminance, ++gaussBufferIndex)
      {
        leftGauss[gaussBufferIndex % 3] = gauss(luminance, theECImage.grayscaled.width);
        int sobelL = gradient(leftGauss, gaussBufferIndex + 2);
        if(sobelL < sobelMin)
        {
          edgeXMin = x;
          sobelMin = sobelL;
        }
      }
      // save region
      ASSERT(leftX < edgeXMin);
      regions.emplace_back(leftX, edgeXMin, getHorizontalRepresentativeValue(theECImage.grayscaled, leftX, edgeXMin, y),
                           getHorizontalRepresentativeHueValue(theECImage.hued, leftX, edgeXMin, y),
                           getHorizontalRepresentativeValue(theECImage.saturated, leftX, edgeXMin, y));
      leftX = edgeXMin;
      regionAdded = true;
      MID_DOT("module:ScanLineRegionizer:horizontalRegionSplit", edgeXMin, y, ColorRGBA::magenta, ColorRGBA::magenta);
      nextRegionWhite = true;
    }
    if(prelabelAsWhite && regionAdded && regions.size() >= 2 && regions[regions.size() - 2].color == Color::white)
    {
      if(!prelabelWhiteNeighborCheck(regions[regions.size() - 2], regions.back()))
        regions[regions.size() - 2].color = Color::black; // black used as starting value
    }
    // prepare next loop
    ++gridLineIndex;
    gridX = nextGridX;
    if(gridLineIndex < theScanGrid.lines.size())
      nextGridX = theScanGrid.lines[gridLineIndex].x;
    else
      nextGridX = theECImage.grayscaled.width - 2;
    gridValue = nextGridValue;
    leftGauss = rightGauss;
  }
  // add last region
  ASSERT(leftX < rightmostX);
  regions.emplace_back(leftX, rightmostX, getHorizontalRepresentativeValue(theECImage.grayscaled, leftX, rightmostX, y),
                       getHorizontalRepresentativeHueValue(theECImage.hued, leftX, rightmostX, y),
                       getHorizontalRepresentativeValue(theECImage.saturated, leftX, rightmostX, y));
}

void ScanLineRegionizer::scanHorizontalAdditionalSmoothing(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const
{
  if(y < 2 || y >= theECImage.grayscaled.height - 2)
    return;
  // initialize variables
  const int scanY = static_cast<int>(y);
  unsigned int leftX = leftmostX;
  const unsigned int scanStop = rightmostX >= 2 ? rightmostX - 2 : 0;
  const int thresholdAdaption = 100;
  int threshold = thresholdAdaption * static_cast<int>(edgeThreshold);
  // define filter
  auto gauss = [](const PixelTypes::GrayscaledPixel* line, const unsigned int imageWidth) // 5x5 gauss smoothing vertical
  {
    return static_cast<int>(line[-2 * static_cast<int>(imageWidth)] + 2 * line[-static_cast<int>(imageWidth)] + 4 * line[0] +
                            2 * line[imageWidth] + line[2 * imageWidth]);
  };
  auto gaussSecond = [](std::array<int, 5>& gaussBuffer, int x) // 5x5 gauss smoothing horizontal
  {
    return gaussBuffer[(x - 2) % 5] + 2 * gaussBuffer[(x - 1) % 5] +
           4 * gaussBuffer[x % 5] + 2 * gaussBuffer[(x + 1) % 5] + gaussBuffer[(x + 2) % 5];
  };
  auto gradient = [](std::array<int, 5>& gaussBuffer, int x)
  {
    return gaussBuffer[(x - 2) % 5] + 2 * gaussBuffer[(x - 1) % 5] - 2 * gaussBuffer[(x + 1) % 5] - gaussBuffer[(x + 2) % 5];
  };
  // Initialize the buffer of smoothed values.
  const PixelTypes::GrayscaledPixel* luminance = &theECImage.grayscaled[scanY][leftX];
  std::array<int, 5> leftGauss{};  // buffer for the left grid point and for sobel scans
  std::array<int, 5> rightGauss{}; // buffer for the right grid point, centered around grid point -> gridX is at index 2
  leftGauss[0] = gauss(luminance, theECImage.grayscaled.width);
  leftGauss[1] = gauss(++luminance, theECImage.grayscaled.width);
  leftGauss[2] = gauss(++luminance, theECImage.grayscaled.width);
  leftGauss[3] = gauss(++luminance, theECImage.grayscaled.width);
  leftGauss[4] = gauss(++luminance, theECImage.grayscaled.width);
  int gaussBufferIndex = 0;
  // grid stuff
  unsigned int gridX = leftX + 2;
  int gridValue = gaussSecond(leftGauss, 2);
  int nextGridValue;
  size_t gridLineIndex = theScanGrid.lines[theScanGrid.lowResStart].x <= 1 ? theScanGrid.lowResStart + theScanGrid.lowResStep : theScanGrid.lowResStart;
  unsigned int nextGridX = theScanGrid.lines[gridLineIndex].x;
  while(nextGridX <= gridX && nextGridX <= scanStop && ++gridLineIndex < theScanGrid.lines.size())
  {
    nextGridX = theScanGrid.lines[gridLineIndex].x;
  }
  if(nextGridX <= gridX && nextGridX < scanStop)
    nextGridX = rightmostX; // skip the loop, make only a single region

  while(gridLineIndex <= theScanGrid.lines.size() && nextGridX <= scanStop)
  {
    rightGauss[0] = gauss(&theECImage.grayscaled[scanY][nextGridX - 2], theECImage.grayscaled.width);
    rightGauss[1] = gauss(&theECImage.grayscaled[scanY][nextGridX - 1], theECImage.grayscaled.width);
    rightGauss[2] = gauss(&theECImage.grayscaled[scanY][nextGridX], theECImage.grayscaled.width);
    rightGauss[3] = gauss(&theECImage.grayscaled[scanY][nextGridX + 1], theECImage.grayscaled.width);
    rightGauss[4] = gauss(&theECImage.grayscaled[scanY][nextGridX + 2], theECImage.grayscaled.width);
    nextGridValue = gaussSecond(rightGauss, 2);
    if(gridValue - nextGridValue >= threshold)
    {
      // find exact edge position
      unsigned int edgeXMax = gridX;
      int sobelMax = gradient(leftGauss, 2);
      luminance = &theECImage.grayscaled[scanY][gridX + 1];
      gaussBufferIndex = 0;
      for(unsigned int x = gridX + 1; x < nextGridX; ++x, ++luminance, ++gaussBufferIndex)
      {
        leftGauss[gaussBufferIndex % 5] = gauss(luminance, theECImage.grayscaled.width);
        int sobelL = gradient(leftGauss, gaussBufferIndex + 3);
        if(sobelL > sobelMax)
        {
          edgeXMax = x;
          sobelMax = sobelL;
        }
      }
      // save region
      ASSERT(leftX < edgeXMax);
      regions.emplace_back(leftX, edgeXMax, getHorizontalRepresentativeValue(theECImage.grayscaled, leftX, edgeXMax, y),
                           getHorizontalRepresentativeHueValue(theECImage.hued, leftX, edgeXMax, y),
                           getHorizontalRepresentativeValue(theECImage.saturated, leftX, edgeXMax, y));
      leftX = edgeXMax;
      MID_DOT("module:ScanLineRegionizer:horizontalRegionSplit", edgeXMax, y, ColorRGBA::magenta, ColorRGBA::magenta);
    }
    else if(gridValue - nextGridValue <= -threshold)
    {
      // find exact edge position
      unsigned int edgeXMin = gridX;
      int sobelMin = gradient(leftGauss, 2);
      luminance = &theECImage.grayscaled[scanY][gridX + 1];
      gaussBufferIndex = 0;
      for(unsigned int x = gridX + 1; x < nextGridX; ++x, ++luminance, ++gaussBufferIndex)
      {
        leftGauss[gaussBufferIndex % 5] = gauss(luminance, theECImage.grayscaled.width);
        int sobelL = gradient(leftGauss, gaussBufferIndex + 3);
        if(sobelL < sobelMin)
        {
          edgeXMin = x;
          sobelMin = sobelL;
        }
      }
      // save region
      ASSERT(leftX < edgeXMin);
      regions.emplace_back(leftX, edgeXMin, getHorizontalRepresentativeValue(theECImage.grayscaled, leftX, edgeXMin, y),
                           getHorizontalRepresentativeHueValue(theECImage.hued, leftX, edgeXMin, y),
                           getHorizontalRepresentativeValue(theECImage.saturated, leftX, edgeXMin, y));
      leftX = edgeXMin;
      MID_DOT("module:ScanLineRegionizer:horizontalRegionSplit", edgeXMin, y, ColorRGBA::magenta, ColorRGBA::magenta);
    }
    // prepare next loop
    gridLineIndex += theScanGrid.lowResStep;
    gridX = nextGridX;
    if(gridLineIndex < theScanGrid.lines.size())
      nextGridX = theScanGrid.lines[gridLineIndex].x;
    else
      nextGridX = theECImage.grayscaled.width - 3;
    gridValue = nextGridValue;
    leftGauss = rightGauss;
  }
  // add last region
  ASSERT(leftX < rightmostX);
  regions.emplace_back(leftX, rightmostX, getHorizontalRepresentativeValue(theECImage.grayscaled, leftX, rightmostX, y),
                       getHorizontalRepresentativeHueValue(theECImage.hued, leftX, rightmostX, y),
                       getHorizontalRepresentativeValue(theECImage.saturated, leftX, rightmostX, y));
}

int ScanLineRegionizer::horizontalScanStart(const int x, const int y) const
{
  ASSERT(theFieldBoundary.isValid);
  int leftX = x;
  int leftY = theFieldBoundary.getBoundaryY(leftX);
  if(leftY <= y)
    return x;
  for(const Vector2i& boundaryPoint : theFieldBoundary.boundaryInImage)
  {
    if(boundaryPoint.x() < leftX)
      continue;
    if(boundaryPoint.x() >= theCameraInfo.width)
      break;
    if(boundaryPoint.y() < y)
    {
      auto xDistance = static_cast<float>(boundaryPoint.x() - leftX);
      auto alpha = static_cast<float>(leftY - y) / static_cast<float>(leftY - boundaryPoint.y());
      return leftX + std::max(static_cast<int>(xDistance * alpha), 0);
    }
    else
    {
      leftX = boundaryPoint.x();
      leftY = boundaryPoint.y();
    }
  }
  int rightY = theFieldBoundary.getBoundaryY(theCameraInfo.width);
  if(rightY < y)
  {
    auto xDistance = static_cast<float>(theCameraInfo.width - leftX);
    auto alpha = static_cast<float>(leftY - y) / static_cast<float>(leftY - rightY);
    return leftX + std::max(static_cast<int>(xDistance * alpha), 0);
  }
  return theCameraInfo.width;
}

int ScanLineRegionizer::horizontalScanStop(const int x, const int y) const
{
  ASSERT(theFieldBoundary.isValid);
  int rightX = x;
  int rightY = theFieldBoundary.getBoundaryY(rightX);
  if(rightY <= y)
    return x;
  auto it = theFieldBoundary.boundaryInImage.rbegin();
  while(it != theFieldBoundary.boundaryInImage.rend())
  {
    if(it->x() >= rightX)
    {
      ++it;
      continue;
    }
    if(it->x() < 0)
      break;
    if(it->y() < y)
    {
      auto xDistance = static_cast<float>(rightX - it->x());
      auto alpha = static_cast<float>(rightY - y) / static_cast<float>(rightY - it->y());
      return rightX - std::max(static_cast<int>(xDistance * alpha), 0) + 1;
    }
    else
    {
      rightX = it->x();
      rightY = it->y();
    }
    ++it;
  }
  int leftY = theFieldBoundary.getBoundaryY(0);
  if(leftY < y)
  {
    auto xDistance = static_cast<float>(rightX);
    auto alpha = static_cast<float>(rightY - y) / static_cast<float>(rightY - leftY);
    return rightX - std::max(static_cast<int>(xDistance * alpha), 0) + 1;
  }
  return 1;
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
  while(theScanGrid.y[gridYIndex] + verticalGridScanMinStep > gridY && gridYIndex + 1 < theScanGrid.y.size())
    ++gridYIndex;
  int nextGridY = theScanGrid.y[gridYIndex];
  if(nextGridY >= gridY)
    nextGridY = 2;

  // start with 5x5 filter
  if(lowestY > middleY)
  {
    auto smoothedGauss = [](const PixelTypes::GrayscaledPixel* line) // 5x5 gauss horizontal
    {
      return static_cast<int>(line[-2] + 2 * line[-1] + 4 * line[0] + 2 * line[1] + line[2]);
    };
    auto smoothedGaussSecond = [](std::array<int, 5>& gaussBuffer, int y) // 5x5 gauss vertical
    {
      return gaussBuffer[(y - 2) % 5] + 2 * gaussBuffer[(y - 1) % 5] + 4 * gaussBuffer[y % 5] +
             2 * gaussBuffer[(y + 1) % 5] + gaussBuffer[(y + 2) % 5];
    };
    auto smoothedGradient = [](std::array<int, 5>& gaussBuffer, int y)
    {
      return gaussBuffer[(y - 2) % 5] + 2 * gaussBuffer[(y - 1) % 5] - 2 * gaussBuffer[(y + 1) % 5] - gaussBuffer[(y + 2) % 5];
    };
    // Initialize the buffer of smoothed values.
    std::array<int, 5> lowerGauss = {}; // buffer for the lower grid point and for sobel scans
    std::array<int, 5> upperGauss = {}; // buffer for the upper grid point, centered around grid point -> gridX is at index 2
    lowerGauss[0] = smoothedGauss(luminance + 2 * theECImage.grayscaled.width);
    lowerGauss[1] = smoothedGauss(luminance + theECImage.grayscaled.width);
    lowerGauss[2] = smoothedGauss(luminance);
    lowerGauss[3] = smoothedGauss(luminance -= theECImage.grayscaled.width);
    lowerGauss[4] = smoothedGauss(luminance -= theECImage.grayscaled.width);
    gridValue = smoothedGaussSecond(lowerGauss, 2);

    while(nextGridY > middleY && gridYIndex <= theScanGrid.y.size())
    {
      upperGauss[0] = smoothedGauss(&theECImage.grayscaled[nextGridY + 2][line.x]);
      upperGauss[1] = smoothedGauss(&theECImage.grayscaled[nextGridY + 1][line.x]);
      upperGauss[2] = smoothedGauss(&theECImage.grayscaled[nextGridY][line.x]);
      upperGauss[3] = smoothedGauss(&theECImage.grayscaled[nextGridY - 1][line.x]);
      upperGauss[4] = smoothedGauss(&theECImage.grayscaled[nextGridY - 2][line.x]);
      nextGridValue = smoothedGaussSecond(upperGauss, 2);
      if(gridValue - nextGridValue >= threshold)
      {
        // find exact edge position
        unsigned int edgeYMax = gridY;
        int sobelMax = smoothedGradient(lowerGauss, 2);
        luminance = &theECImage.grayscaled[gridY - 1][line.x];
        int gaussBufferIndex = 0;
        for(int y = gridY - 1; y > nextGridY; --y, ++gaussBufferIndex, luminance -= theECImage.grayscaled.width)
        {
          lowerGauss[gaussBufferIndex % 5] = smoothedGauss(luminance);
          int sobelL = smoothedGradient(lowerGauss, gaussBufferIndex + 3);
          if(sobelL > sobelMax)
          {
            edgeYMax = y;
            sobelMax = sobelL;
          }
        }
        // save region
        ASSERT(lowerY > edgeYMax);
        regions.emplace_back(edgeYMax, lowerY, getVerticalRepresentativeValue(theECImage.grayscaled, line.x, edgeYMax, lowerY),
                             getVerticalRepresentativeHueValue(theECImage.hued, line.x, edgeYMax, lowerY),
                             getVerticalRepresentativeValue(theECImage.saturated, line.x, edgeYMax, lowerY));
        lowerY = edgeYMax;
        MID_DOT("module:ScanLineRegionizer:verticalRegionSplit", line.x, edgeYMax, ColorRGBA::magenta, ColorRGBA::magenta);
      }
      else if(gridValue - nextGridValue <= -threshold)
      {
        // find exact edge position
        unsigned int edgeYMin = gridY;
        int sobelMin = smoothedGradient(lowerGauss, 2);
        luminance = &theECImage.grayscaled[gridY - 1][line.x];
        int gaussBufferIndex = 0;
        for(int y = gridY - 1; y > nextGridY; --y, ++gaussBufferIndex, luminance -= theECImage.grayscaled.width)
        {
          lowerGauss[gaussBufferIndex % 5] = smoothedGauss(luminance);
          int sobelL = smoothedGradient(lowerGauss, gaussBufferIndex + 3);
          if(sobelL < sobelMin)
          {
            edgeYMin = y;
            sobelMin = sobelL;
          }
        }
        // save region
        ASSERT(lowerY > edgeYMin);
        regions.emplace_back(edgeYMin, lowerY, getVerticalRepresentativeValue(theECImage.grayscaled, line.x, edgeYMin, lowerY),
                             getVerticalRepresentativeHueValue(theECImage.hued, line.x, edgeYMin, lowerY),
                             getVerticalRepresentativeValue(theECImage.saturated, line.x, edgeYMin, lowerY));
        lowerY = edgeYMin;
        MID_DOT("module:ScanLineRegionizer:verticalRegionSplit", line.x, edgeYMin, ColorRGBA::magenta, ColorRGBA::magenta);
      }
      // prepare next loop
      gridY = nextGridY;
      while(gridY < nextGridY + verticalGridScanMinStep)
      {
        ++gridYIndex;
        if(gridYIndex < theScanGrid.y.size())
          nextGridY = theScanGrid.y[gridYIndex];
        else
        {
          nextGridY = 2;
          continue;
        }
      }
      gridValue = nextGridValue;
      lowerGauss = upperGauss;
    }
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
    std::array<int, 3> gaussBuffer{};
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
            regions.back().color = Color::white;
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
        if(prelabelAsWhite && regions.size() >= 2 && regions[regions.size() - 2].color == Color::white)
        {
          if(!prelabelWhiteNeighborCheck(regions[regions.size() - 2], regions.back()))
            regions[regions.size() - 2].color = Color::black; // black used as starting value
        }
        gridY = nextGridY;
        while(gridY < nextGridY + verticalGridScanMinStep)
        {
          ++gridYIndex;
          if(gridYIndex < theScanGrid.y.size())
            nextGridY = theScanGrid.y[gridYIndex];
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
                       getVerticalRepresentativeHueValue(theECImage.hued, line.x, top, lowerY), getVerticalRepresentativeValue(theECImage.saturated, line.x, top, lowerY));
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
    // build list of regions in the neighbourhood of the line. Can span multiple scan lines due to ScanGrid layout
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
          nextLinesRegions.emplace_back(std::make_pair(&regions[nextLineIndex][nextLineRegionIndex], x[nextLineIndex]));
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
  // test for white because of white prelabeling
  if(std::abs(static_cast<char>(aParent->y) - static_cast<char>(bParent->y)) < luminanceSimilarityThreshold &&
     std::abs(static_cast<char>(aParent->h) - static_cast<char>(bParent->h)) < hueSimilarityThreshold &&
     std::abs(static_cast<char>(aParent->s) - static_cast<char>(bParent->s)) < saturationSimilarityThreshold &&
     aParent->color != Color::white && bParent->color != Color::white)
    return true;
  return false;
}

void ScanLineRegionizer::classifyFieldRegions(const std::vector<unsigned short>& xy, std::vector<std::vector<InternalRegion>>& regions, bool horizontal)
{
  unsigned char minHue = 255;
  unsigned char maxHue = 0;
  unsigned char minSaturation = 138;
  // luminance is only important for images with low saturated field
  unsigned char maxLuminance = baseLuminance;
  int dataPoints = 0;
  for(auto& line : regions)
  {
    for(auto& region : line)
    {
      InternalRegion* representative = region.findSet();
      // ASSERTS that all Regions start with color == Color::black !
      if(representative->color == Color::black)
      {
        if(regionIsField(representative))
        {
          representative->color = Color::field;
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
          representative->color = Color::none;
      }
      region.color = representative->color;
    }
  }
  if(dataPoints > 0)
  {
    // expand min/max range
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
      classifyFieldHorizontal(xy, regions);
    else
      classifyFieldVertical(regions);
  }
  COMPLEX_DRAWING("module:ScanLineRegionizer:fieldColorRange")
  {
    int fontSize = 9;
    if(theCameraInfo.camera == CameraInfo::upper)
      fontSize = 14;
    int yPos = fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRange", 10, yPos, fontSize, ColorRGBA::orange, "MinHue: " << estimatedFieldColor.minHue);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRange", 10, yPos, fontSize, ColorRGBA::orange, "MaxHue: " << estimatedFieldColor.maxHue);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRange", 10, yPos, fontSize, ColorRGBA::orange, "MinSat: " << estimatedFieldColor.minSaturation);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRange", 10, yPos, fontSize, ColorRGBA::orange, "MaxLum: " << estimatedFieldColor.maxLuminance);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRange", 10, yPos, fontSize, ColorRGBA::orange, "DataPoints: " << dataPoints);
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
      // saturation of very dark field pixels is often 0
      unsigned char minSaturation = 0;
      if(region.y > 2)
      {
        if(theCameraInfo.camera == CameraInfo::upper)
        {
          // lower saturation allowed for further away regions
          float scalingFactor = (static_cast<float>(y[lineIndex] - theScanGrid.fieldLimit) /
                                 static_cast<float>(theCameraInfo.height - theScanGrid.fieldLimit) + 1.f) / 2.f;
          minSaturation = static_cast<unsigned char>(static_cast<float>(estimatedFieldColor.minSaturation) * scalingFactor);
        }
        else
          minSaturation = estimatedFieldColor.minSaturation;
      }
      char hueRangeExpansion = 0;
      if(theCameraInfo.camera == CameraInfo::upper)
        hueRangeExpansion = static_cast<char>(0.5f * static_cast<float>(hueSimilarityThreshold) - static_cast<float>(y[lineIndex] - theScanGrid.fieldLimit) /
                                              static_cast<float>(theCameraInfo.height - theScanGrid.fieldLimit) * 0.5f * static_cast<float>(hueSimilarityThreshold));
      if(region.color != Color::white)
      {
        if(region.h >= estimatedFieldColor.minHue - hueRangeExpansion &&
           region.h <= estimatedFieldColor.maxHue + hueRangeExpansion &&
           region.s >= minSaturation && region.y <= estimatedFieldColor.maxLuminance)
          region.color = Color::field;
        else
          region.color = Color::none;
      }
    }
  }
}

void ScanLineRegionizer::classifyFieldVertical(std::vector<std::vector<InternalRegion>>& regions) const
{
  for(auto& line : regions)
  {
    for(auto& region : line)
    {
      // saturation of very dark field pixels is often 0
      unsigned char minSaturation = 0;
      if(region.y > 2)
      {
        if(theCameraInfo.camera == CameraInfo::upper)
        {
          // lower saturation allowed for further away regions
          float scalingFactor = (static_cast<float>(region.range.from - theScanGrid.fieldLimit) /
                                 static_cast<float>(theCameraInfo.height - theScanGrid.fieldLimit) + 1.f) / 2.f;
          minSaturation = static_cast<unsigned char>(static_cast<float>(estimatedFieldColor.minSaturation) * scalingFactor);
        }
        else
          minSaturation = estimatedFieldColor.minSaturation;
      }
      char hueRangeExpansion = 0;
      if(theCameraInfo.camera == CameraInfo::upper)
        hueRangeExpansion = static_cast<char>(0.5f * static_cast<float>(hueSimilarityThreshold) - static_cast<float>(region.range.from - theScanGrid.fieldLimit) /
                            static_cast<float>(theCameraInfo.height - theScanGrid.fieldLimit) * 0.5f * static_cast<float>(hueSimilarityThreshold));
      if(region.color != Color::white)
      {
        if(region.h >= estimatedFieldColor.minHue - hueRangeExpansion &&
           region.h <= estimatedFieldColor.maxHue + hueRangeExpansion &&
           region.s >= minSaturation && region.y <= estimatedFieldColor.maxLuminance)
          region.color = Color::field;
        else
          region.color = Color::none;
      }
    }
  }
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
    int xPos = 240;
    if(theCameraInfo.camera == CameraInfo::upper)
    {
      fontSize = 14;
      xPos = 320;
    }
    int yPos = fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:whiteRange", xPos, yPos, fontSize, ColorRGBA::orange, "MinLum: " << minWhiteLuminance);
    yPos += fontSize + 2;
    DRAW_TEXT("module:ScanLineRegionizer:fieldColorRange", xPos, yPos, fontSize, ColorRGBA::orange, "MaxSat: " << maxWhiteSaturation);
  }
  for(auto& line : regions)
  {
    for(auto& region : line)
    {
      if(region.color != Color::field && region.y > minWhiteLuminance && region.s < maxWhiteSaturation)
        region.color = Color::white;
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
  Color lastColor = Color::none;
  Color currentColor = Color::none;
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
      if(currentColor == Color::none && currentRegionSize <= maxRegionSizeForStitching)
      {
        if(line[regionIndex].color == Color::none && regionIndex + 1 < line.size() &&
           currentRegionSize + line[regionIndex].range.to - line[regionIndex].range.from <= maxRegionSizeForStitching)
        {
          // Successive "none" regions that can be changed together
          currentRegionSize += line[regionIndex].range.to - line[regionIndex].range.from;
          continue;
        }
        if(lastColor == Color::field && line[regionIndex].color == Color::field)
        {
          for(size_t changeIndex = lastColorIndex + 1; changeIndex < regionIndex; ++changeIndex)
            line[changeIndex].color = Color::field;
        }
        else if((lastColor == Color::field && line[regionIndex].color == Color::white) ||
                (lastColor == Color::white && line[regionIndex].color == Color::field))
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

bool ScanLineRegionizer::isEstimatedFieldColorValid() const
{
  return theFrameInfo.getTimeSince(estimatedFieldColor.lastSet) <= estimatedFieldColorInvalidationTime;
}

PixelTypes::GrayscaledPixel ScanLineRegionizer::getHorizontalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image,
                                                                                 const unsigned int from, const unsigned int to, const unsigned int y)
{
  ASSERT(to > from);
  const unsigned int length = to - from;
  unsigned int sum = 0;
  if(length <= 6)
  {
    for(unsigned int x = from; x < to; ++x) // take every pixel from very small regions
      sum += image[y][x];
    return static_cast<unsigned char>(sum / length);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle sized regions
    for(unsigned int x = from + 1; x < to - 1; x += 2) // don't start or stop in the edge of the region
      sum += image[y][x];
    return static_cast<unsigned char>(sum / ((length - 1) / 2));
  }
  // take every fourth pixel for bigger regions
  for(unsigned int x = from + 2; x < to - 1; x += 4) // don't start or stop in the edge of the region
    sum += image[y][x];
  // this relies on (length / 4) being floored; expanding to 4*sum/length will lead to false results
  return static_cast<unsigned char>(sum / (length / 4));
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
    for(unsigned int x = from; x < to; ++x, ++currentStep) // take every pixel from very small regions
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
    return static_cast<unsigned char>(hueValue);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle sized regions
    for(unsigned int x = from + 1; x < to - 1; x += 2, ++currentStep) // don't start or stop in the edge of the region
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
    return static_cast<unsigned char>(hueValue);
  }
  // take every fourth pixel for bigger regions
  for(unsigned int x = from + 2; x < to - 1; x += 4, ++currentStep) // don't start or stop in the edge of the region
    hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
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
    for(unsigned int y = from; y < to; ++y) // take every pixel from very small regions
      sum += image[y][x];
    return static_cast<unsigned char>(sum / length);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle sized regions
    for(unsigned int y = from + 1; y < to - 1; y += 2) // don't start or stop in the edge of the region
      sum += image[y][x];
    return static_cast<unsigned char>(sum / ((length - 1) / 2));
  }
  for(unsigned int y = from + 2; y < to - 1; y += 4) // don't start or stop in the edge of the region
    sum += image[y][x];
  // this relies on (length / 4) being floored, so please don't change to 4*sum/length as that will lead to false results
  return static_cast<unsigned char>(sum / (length / 4));
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
    for(unsigned int y = from; y < to; ++y, ++currentStep) // take every pixel from very small regions
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
    return static_cast<unsigned char>(hueValue);
  }
  if(length <= 19)
  {
    // take every second pixel from small to middle sized regions
    for(unsigned int y = from + 1; y < to - 1; y += 2, ++currentStep) // don't start or stop in the edge of the region
      hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
    return static_cast<unsigned char>(hueValue);
  }
  for(unsigned int y = from + 2; y < to - 1; y += 4, ++currentStep) // don't start or stop in the edge of the region
    hueValue = ScanLineRegionizer::hueAverage(hueValue, image[y][x], currentStep);
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
  float hueDiff = hueAddition - hueValue;
  if(hueDiff >= 128.f)
  {
    hueValue += (hueDiff - 256.f) / static_cast<float>(dataPoints);
    if(hueValue < 0.f)
      hueValue += 256.f;
  }
  else if(hueDiff <= -128.f)
  {
    hueValue += (hueDiff + 256.f) / static_cast<float>(dataPoints);
    if(hueValue >= 256.f)
      hueValue -= 256.f;
  }
  else
    hueValue += hueDiff / static_cast<float>(dataPoints);
  return hueValue;
}
