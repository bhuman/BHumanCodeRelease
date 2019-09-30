/**
 * The file declares a module that provides the description of a grid for scanning
 * the image. The grid resolution adapts to the camera perspective.
 * @author Thomas Röfer
 */

#include "ScanGridProvider.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>

MAKE_MODULE(ScanGridProvider, perception)

void ScanGridProvider::update(ScanGrid& scanGrid)
{
  scanGrid.y.clear();
  scanGrid.lines.clear();

  if(!theCameraMatrix.isValid)
    return; // Cannot compute grid without camera matrix

  // Compute the furthest point away that could be part of the field given an unknown own position.
  Vector2f pointInImage;
  const float fieldDiagonal = Vector2f(theFieldDimensions.boundary.x.getSize(), theFieldDimensions.boundary.y.getSize()).norm();
  if(!Transformation::robotWithCameraRotationToImage(Vector2f(fieldDiagonal, 0), theCameraMatrix, theCameraInfo, pointInImage))
    return; // Cannot project furthest possible point to image -> no grid in image

  scanGrid.fieldLimit = std::max(static_cast<int>(pointInImage.y()), -1);
  if(scanGrid.fieldLimit >= theCameraInfo.height)
    return; // Image is above field limit -> no grid in image

  // Determine the maximum distance between scan lines at the bottom of the image not to miss the ball.
  Vector2f leftOnField;
  Vector2f rightOnField;
  if(!Transformation::imageToRobotWithCameraRotation(Vector2i(0, theCameraInfo.height - 1), theCameraMatrix, theCameraInfo, leftOnField) ||
     !Transformation::imageToRobotWithCameraRotation(Vector2i(theCameraInfo.width, theCameraInfo.height - 1), theCameraMatrix, theCameraInfo, rightOnField))
    return; // Cannot project lower image border to field -> no grid

  const int xStepUpperBound = theCameraInfo.width / minNumOfLowResScanLines;
  const int maxXStep = std::min(xStepUpperBound,
                                static_cast<int>(theCameraInfo.width * theBallSpecification.radius * 2.f *
                                                 ballWidthRatio / (leftOnField - rightOnField).norm()));
  Vector2f pointOnField = (leftOnField + rightOnField) / 2.f;

  // Determine vertical sampling points of the grid
  scanGrid.y.reserve(theCameraInfo.height);
  const float fieldStep = theFieldDimensions.fieldLinesWidth * lineWidthRatio;
  bool singleSteps = false;
  int y;
  for(y = theCameraInfo.height - 1; y > scanGrid.fieldLimit;)
  {
    scanGrid.y.emplace_back(y);
    // Calc next vertical position for all scan lines.
    if(singleSteps)
      --y;
    else
    {
      pointOnField.x() += fieldStep;
      if(!Transformation::robotWithCameraRotationToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
        break;
      const int y2 = y;
      y = std::min(y2 - 1, static_cast<int>(pointInImage.y() + 0.5));
      singleSteps = y2 - 1 == y;
    }
  }
  if(y < 0 && !scanGrid.y.empty() && scanGrid.y.back() != 0)
    scanGrid.y.emplace_back(0);

  // Determine the maximum distance between scan lines at the top of the image not to miss the ball. Do not go below minStepSize.
  int minXStep = minStepSize;
  if(Transformation::imageToRobotWithCameraRotation(Vector2i(0, 0), theCameraMatrix, theCameraInfo, leftOnField) &&
     Transformation::imageToRobotWithCameraRotation(Vector2i(theCameraInfo.width, 0), theCameraMatrix, theCameraInfo, rightOnField))
    minXStep = std::max(minXStep, static_cast<int>(theCameraInfo.width * theBallSpecification.radius *
                                                   2.f * ballWidthRatio / (leftOnField - rightOnField).norm()));
  minXStep = std::min(xStepUpperBound, minXStep);

  // Determine a max step size that fulfills maxXStep2 = minXStep * 2^n, maxXStep2 <= maxXStep.
  // Also compute lower y coordinates for the different lengths of scan lines.
  int maxXStep2 = minXStep;
  std::vector<int> yStarts;
  while(maxXStep2 * 2 <= maxXStep)
  {
    float distance = Projection::getDistanceBySize(theCameraInfo, theBallSpecification.radius * ballWidthRatio, static_cast<float>(maxXStep2));
    VERIFY(Transformation::robotWithCameraRotationToImage(Vector2f(distance, 0), theCameraMatrix, theCameraInfo, pointInImage));
    yStarts.push_back(static_cast<int>(pointInImage.y() + 0.5f));
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
  scanGrid.lines.reserve((theCameraInfo.width - xStart) / minXStep);
  size_t i = yStarts2.size() / 2; // Start with the second longest scan line.
  for(int x = xStart; x < theCameraInfo.width; x += minXStep)
  {
    int yMax = std::min(yStarts2[i++], theCameraInfo.height);
    i %= yStarts2.size();
    theBodyContour.clipBottom(x, yMax);
    yMax = std::max(0, yMax);
    const size_t yMaxIndex = std::upper_bound(scanGrid.y.begin(), scanGrid.y.end(), yMax + 1, std::greater<int>()) - scanGrid.y.begin();
    scanGrid.lines.emplace_back(x, yMax, static_cast<unsigned>(yMaxIndex));
  }

  // Set low resolution scan line info
  scanGrid.lowResStep = maxXStep2 / minXStep;
  scanGrid.lowResStart = scanGrid.lowResStep / 2;
}
