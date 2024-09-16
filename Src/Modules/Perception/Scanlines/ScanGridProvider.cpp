/**
 * The file implements a module that provides the description of a grid for scanning
 * the image. The grid resolution adapts to the camera perspective.
 * @author Thomas Röfer
 * @author Lukas Malte Monnerjahn
 */

#include "ScanGridProvider.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(ScanGridProvider);

void ScanGridProvider::update(ScanGrid& scanGrid)
{
  scanGrid.clear();

  if(!theCameraMatrix.isValid || !theFieldBoundary.isValid)
    return; // Cannot compute grid without camera matrix

  scanGrid.fieldLimit = calcFieldLimit();
  if(scanGrid.fieldLimit < 0)
    return;

  ImageCornersOnField lowerImageCornersOnField = calcImageCornersOnField(VerticalBoundary::LOWER);
  if(!lowerImageCornersOnField.valid)
    return; // Cannot project lower image border to field -> no grid

  setFullResY(scanGrid, lowerImageCornersOnField);
  if(!scanGrid.fullResY.empty())
  {
    setLowResHorizontalLines(scanGrid);
    setVerticalLines(scanGrid, lowerImageCornersOnField);
  }
}

int ScanGridProvider::calcFieldLimit() const
{
  Vector2f pointInImage;
  const float fieldDiagonal = Vector2f(theFieldDimensions.boundary.x.getSize(), theFieldDimensions.boundary.y.getSize()).norm();
  if(!Transformation::robotWithCameraRotationToImage(Vector2f(fieldDiagonal, 0), theCameraMatrix, theCameraInfo, pointInImage))
    return -1; // Cannot project furthest possible point to image -> no grid in image

  // If upper image border is below fieldLimit, set it to the upper border
  int fieldLimit = std::max(static_cast<int>(pointInImage.y()), 0);
  if(fieldLimit >= theCameraInfo.height)
    return -1; // Image is above field limit -> no grid in image
  return fieldLimit;
}

ScanGridProvider::ImageCornersOnField ScanGridProvider::calcImageCornersOnField(VerticalBoundary boundary) const
{
  ImageCornersOnField imageCornersOnField;
  imageCornersOnField.valid = false;
  Vector2i leftCorner(0, boundary == VerticalBoundary::UPPER ? 0 : theCameraInfo.height - 1);
  Vector2i rightCorner(theCameraInfo.width, boundary == VerticalBoundary::UPPER ? 0 : theCameraInfo.height - 1);
  if(Transformation::imageToRobotWithCameraRotation(
      leftCorner, theCameraMatrix, theCameraInfo, imageCornersOnField.leftOnField) &&
     Transformation::imageToRobotWithCameraRotation(
         rightCorner, theCameraMatrix, theCameraInfo, imageCornersOnField.rightOnField))
  {
    imageCornersOnField.valid = true;
  }
  return imageCornersOnField;
}

void ScanGridProvider::setFullResY(ScanGrid& scanGrid, ScanGridProvider::ImageCornersOnField& lowerImageCornersOnField) const
{
  Vector2f verticalViewCenterPointOnField = (lowerImageCornersOnField.leftOnField + lowerImageCornersOnField.rightOnField) / 2.f;
  scanGrid.fullResY.reserve(theCameraInfo.height);
  const float fieldStep = theFieldDimensions.fieldLinesWidth * lineWidthRatio;
  bool singleSteps = false;
  int y;
  for(y = theCameraInfo.height - 1; y > scanGrid.fieldLimit;)
  {
    scanGrid.fullResY.emplace_back(y);
    if(singleSteps)
      --y;
    else
    {
      verticalViewCenterPointOnField.x() += fieldStep;
      Vector2f verticalViewCenterPointInImage;
      if(!Transformation::robotWithCameraRotationToImage(verticalViewCenterPointOnField, theCameraMatrix, theCameraInfo, verticalViewCenterPointInImage))
        break;
      const int y2 = y;
      y = std::min(y2 - 1, static_cast<int>(std::lround(verticalViewCenterPointInImage.y())));
      singleSteps = y2 - 1 == y;
    }
  }
  if(y < 0 && !scanGrid.fullResY.empty() && scanGrid.fullResY.back() != 0)
    scanGrid.fullResY.emplace_back(0);
}

void ScanGridProvider::setLowResHorizontalLines(ScanGrid& scanGrid) const
{
  scanGrid.lowResHorizontalLines.reserve((theCameraInfo.height / minHorizontalLowResStepSize) + 1);
  bool minSteps = false;
  size_t fullResIndex = 0;
  for(int y = scanGrid.fullResY[fullResIndex]; y > scanGrid.fieldLimit;)
  {
    addLowResHorizontalLine(scanGrid, y);
    if(minSteps)
      y -= minHorizontalLowResStepSize;
    else
    {
      ++fullResIndex;
      if(fullResIndex >= scanGrid.fullResY.size())
        break;
      const int y2 = y;
      y = std::min(y2 - minHorizontalLowResStepSize, scanGrid.fullResY[fullResIndex]);
      minSteps = y2 - minHorizontalLowResStepSize == y;
    }
  }
}

void ScanGridProvider::setVerticalLines(ScanGrid& scanGrid, ImageCornersOnField& lowerImageCornersOnField) const
{
  // Determine the maximum distance between scan lines at the bottom of the image not to miss the ball.
  const int xStepUpperBound = theCameraInfo.width / minNumOfLowResScanLines;
  const int maxXStep = std::min(xStepUpperBound,
                                static_cast<int>(static_cast<float>(theCameraInfo.width) * theBallSpecification.radius * 2.f *
                                                 ballWidthRatio / (lowerImageCornersOnField.leftOnField - lowerImageCornersOnField.rightOnField).norm()));

  // Determine the maximum distance between scan lines at the top of the image not to miss the ball. Do not go below minVerticalStepSize.
  int minXStep = minVerticalStepSize;
  ImageCornersOnField upperImageCornersOnField = calcImageCornersOnField(VerticalBoundary::UPPER);
  if(upperImageCornersOnField.valid)
  {
    minXStep = std::max(
        minXStep,
        static_cast<int>(static_cast<float>(theCameraInfo.width) * theBallSpecification.radius * 2.f *
            ballWidthRatio / (upperImageCornersOnField.leftOnField - upperImageCornersOnField.rightOnField).norm()));
  }
  minXStep = std::min(xStepUpperBound, minXStep);

  // Determine a max step size that fulfills maxXStep2 = minXStep * 2^n, maxXStep2 <= maxXStep.
  // Also compute lower y coordinates for the different lengths of scan lines.
  int maxXStep2 = minXStep;
  std::vector<int> yStarts;
  Vector2f pointInImage;
  while(maxXStep2 * 2 <= maxXStep)
  {
    float distance = Projection::getDistanceBySize(
        theCameraInfo,
        theBallSpecification.radius * ballWidthRatio, static_cast<float>(maxXStep2));
    VERIFY(Transformation::robotWithCameraRotationToImage(Vector2f(distance, 0), theCameraMatrix, theCameraInfo, pointInImage));
    yStarts.push_back(static_cast<int>(std::lround(pointInImage.y())));
    maxXStep2 *= 2;
  }
  yStarts.push_back(theCameraInfo.height);

  // Determine a pattern with the different lengths of scan lines, in which the longest appears once,
  // the second longest twice, etc. The pattern starts with the longest.
  std::vector<int> yStarts2(maxXStep2 / minXStep);
  for(size_t i = 0, step = 1; i < yStarts.size(); ++i, step *= 2)
    for(size_t j = 0; j < yStarts2.size(); j += step)
      yStarts2[j] = yStarts[i];

  // Initialize the scan states and the regions.
  const int xStart = theCameraInfo.width % (theCameraInfo.width / minXStep - 1) / 2;
  scanGrid.verticalLines.reserve((theCameraInfo.width - xStart) / minXStep);
  size_t i = yStarts2.size() / 2; // Start with the second-longest scan line.
  for(int x = xStart; x < theCameraInfo.width; x += minXStep)
  {
    int yMin = std::max(scanGrid.fieldLimit, theFieldBoundary.getBoundaryY(x));
    int yMax = std::min(yStarts2[i++], theCameraInfo.height);
    theBodyContour.clipBottom(x, yMax);
    yMax = std::max(1, yMax);
    yMin = std::min(yMin, yMax - 1);
    i %= yStarts2.size();
    const size_t yMaxIndexUpperBound = std::upper_bound(scanGrid.fullResY.cbegin(), scanGrid.fullResY.cend(), yMax, std::greater_equal<>())
        - scanGrid.fullResY.cbegin();
    const size_t yMaxIndex = std::min(yMaxIndexUpperBound, scanGrid.fullResY.size() - 1);
    auto greaterEqual = [](int value, const ScanGrid::HorizontalLine& hline){return value > hline.y;};
    const size_t lowResYMaxIndexUpperBound = std::upper_bound(scanGrid.lowResHorizontalLines.cbegin(), scanGrid.lowResHorizontalLines.cend(), yMax, greaterEqual)
        - scanGrid.lowResHorizontalLines.cbegin();
    const size_t lowResYMaxIndex = scanGrid.lowResHorizontalLines.empty() ? 0 :
                                   std::min(lowResYMaxIndexUpperBound, scanGrid.lowResHorizontalLines.size() - 1);
    scanGrid.verticalLines.emplace_back(x, yMin, yMax, static_cast<unsigned>(lowResYMaxIndex), static_cast<unsigned>(yMaxIndex));
  }

  // Set low resolution scan line info
  scanGrid.lowResStep = maxXStep2 / minXStep;
  scanGrid.lowResStart = scanGrid.lowResStep / 2;
}

void ScanGridProvider::addLowResHorizontalLine(ScanGrid& scanGrid, int y) const
{
  int left = horizontalLeftScanStop(y);
  int right = horizontalRightScanStop(y);
  if(right > left)
    scanGrid.lowResHorizontalLines.emplace_back(y, left, right);
}

int ScanGridProvider::horizontalLeftScanStop(int usedY) const
{
  // most of the time everything on the left side of the body contour right-side edge is inside the body
  const int bodyScanStart = theBodyContour.getRightEdge(usedY, theCameraInfo.width);
  ASSERT(bodyScanStart >= 0 && bodyScanStart < theCameraInfo.width);
  int boundaryScanStart = horizontalFieldBoundaryLeftScanStop(bodyScanStart, usedY);
  if(boundaryScanStart >= theCameraInfo.width) // may happen due to rounding
    boundaryScanStart = 0;
  return std::max(bodyScanStart, boundaryScanStart);
}

int ScanGridProvider::horizontalRightScanStop(int usedY) const
{
  // most of the time everything on the right side of the body contour left-side edge is inside the body
  const int bodyRight = theBodyContour.getLeftEdge(usedY, theCameraInfo.width);
  ASSERT(bodyRight > 0 && bodyRight <= theCameraInfo.width);
  int boundaryRight = horizontalFieldBoundaryRightScanStop(static_cast<int>(bodyRight), usedY);
  if(boundaryRight <= 0) // may happen due to rounding
    boundaryRight = theCameraInfo.width;
  return std::min(bodyRight, boundaryRight);
}

int ScanGridProvider::horizontalFieldBoundaryLeftScanStop(int x, int y) const
{
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

int ScanGridProvider::horizontalFieldBoundaryRightScanStop(int x, int y) const
{
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
