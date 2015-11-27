/**
 * @file RealisticBallPerceptor.cpp
 * @author Felix Thielke
 */

#include "RealisticBallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"

#include <algorithm>

MAKE_MODULE(RealisticBallPerceptor, perception)

void RealisticBallPerceptor::update(RealisticBallPercepts& ballPercepts)
{
  DECLARE_DEBUG_DRAWING("module:RealisticBallPerceptor:Window", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RealisticBallPerceptor:CirclePoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RealisticBallPerceptor:Circle", "drawingOnImage");

  if(windowRadius > MAX_WINDOW_RADIUS)
  {
    windowRadius = MAX_WINDOW_RADIUS;
  }

  STOPWATCH("RealisticBallPerceptor:edges")
    findEdges();

  COMPLEX_IMAGE(sobelImage)
  {
    INIT_DEBUG_IMAGE_BLACK(sobelImage, width, height);
    for(unsigned short y = 0; y < rows; y++)
    {
      for(const unsigned short& x : points[y])
      {
        DEBUG_IMAGE_SET_PIXEL_YUV(sobelImage, x, y + yStart, 128, 128, 128);
      }
    }
    if(!randomness)
    {
      for(const RealisticBallPerceptor::Point& anchor : anchors)
      {
        DEBUG_IMAGE_SET_PIXEL_YUV(sobelImage, anchor.x, anchor.y + yStart, 128, 0, 128);
      }
    }
  }
  SEND_DEBUG_IMAGE(sobelImage);

  std::vector<RealisticBallPerceptor::Circle> circles;
  if(!randomness && edgeDetectMode == EdgeDetectMode::edgeDrawing)
  {
    STOPWATCH("RealisticBallPerceptor:FRHT")
      fastHT(
        circles,
        windowPadding,
        theCameraInfo.camera == CameraInfo::lower ? minRadiusLower : minRadiusUpper,
        theCameraInfo.camera == CameraInfo::lower ? maxRadiusLower : maxRadiusUpper,
        circleThreshold,
        mergeRadius,
        sqr(mergeDistance)
        );
  }
  else
  {
    STOPWATCH("RealisticBallPerceptor:FRHT")
      fastRHT(
        circles,
        windowPadding,
        theCameraInfo.camera == CameraInfo::lower ? expectedPixelsPerCircleLower : expectedPixelsPerCircleUpper,
        theCameraInfo.camera == CameraInfo::lower ? minRadiusLower : minRadiusUpper,
        theCameraInfo.camera == CameraInfo::lower ? maxRadiusLower : maxRadiusUpper,
        circleThreshold,
        mergeRadius,
        sqr(mergeDistance)
        );
  }

  ballPercepts.balls.clear();
  const float minDistanceFromLineSquared = sqr(minDistanceFromLine);
  const float centerCircleRadiusSquared = sqr(theFieldDimensions.centerCircleRadius);
  for(const RealisticBallPerceptor::Circle& circle : circles)
  {
    RealisticBallPercept ball;
    ball.positionInImage = circle.center;
    ball.radiusInImage = circle.radius;
    STOPWATCH("RealisticBallPerceptor:validate") do
    {
      if((!validatePre || validatePerceptPreField(ball)) && calculateBallOnField(ball) &&
         (!validatePost || validatePerceptPostField(ball, minDistanceFromLineSquared, centerCircleRadiusSquared)))
      {
        ballPercepts.balls.push_back(ball);
      }
    }
    while(false);
  }
}

bool RealisticBallPerceptor::calculateBallOnField(RealisticBallPercept& ballPercept) const
{
  Vector2f leftPosition = theImageCoordinateSystem.toCorrected(Vector2f(ballPercept.positionInImage.x() - ballPercept.radiusInImage, ballPercept.positionInImage.y()));
  Vector2f rightPosition = theImageCoordinateSystem.toCorrected(Vector2f(ballPercept.positionInImage.x() + ballPercept.radiusInImage, ballPercept.positionInImage.y()));
  if(Transformation::imageToRobotHorizontalPlane(leftPosition, expectedRadius, theCameraMatrix, theCameraInfo, leftPosition) &&
     Transformation::imageToRobotHorizontalPlane(rightPosition, expectedRadius, theCameraMatrix, theCameraInfo, rightPosition))
  {
    ballPercept.radiusOnField = (leftPosition - rightPosition).norm() / 2.0f;
    if(Transformation::imageToRobotHorizontalPlane(theImageCoordinateSystem.toCorrected(ballPercept.positionInImage),
                                                   ballPercept.radiusOnField, theCameraMatrix, theCameraInfo, ballPercept.relativePositionOnField))
    {
      ballPercept.absolutePositionOnField = theRobotPose * ballPercept.relativePositionOnField;
      return !validateFieldRadius ||
        (ballPercept.radiusOnField >= minRadiusOnField &&
         ballPercept.radiusOnField <= maxRadiusOnField);
    }
  }

  return false;
}

bool RealisticBallPerceptor::validatePerceptPostField(const RealisticBallPercept& ball, float minDistanceFromLineSquared, float centerCircleRadiusSquared) const
{
  LinePercept::Line l;
  return theFieldDimensions.isInsideField(ball.absolutePositionOnField) &&
    ball.relativePositionOnField.x() >= minDistance &&
    (!theColorTable[theImage[static_cast<int>(ball.positionInImage.y())][static_cast<int>(ball.positionInImage.x())]].is(ColorClasses::white) ||
     std::abs(theFieldDimensions.fieldLinesWidth - ball.radiusOnField * 2) >= minDiffFromLineWidth ||
     ((ball.absolutePositionOnField - theFieldDimensions.fieldLinesWithGoalFrame.getClosestPoint(ball.absolutePositionOnField)).squaredNorm() >= minDistanceFromLineSquared &&
      theLinePercept.getClosestLine(ball.relativePositionOnField, l) >= minDistanceFromLine &&
      std::abs(ball.absolutePositionOnField.squaredNorm() - centerCircleRadiusSquared) >= minDistanceFromLineSquared &&
      std::abs(ball.absolutePositionOnField.squaredNorm() - centerCircleRadiusSquared) >= minDistanceFromLineSquared &&
      (std::abs(ball.absolutePositionOnField.y()) >= minDistanceFromPenaltyMark ||
       std::abs(ball.absolutePositionOnField.x() - theFieldDimensions.xPosOpponentPenaltyMark) >= minDistanceFromPenaltyMark)));
}

bool RealisticBallPerceptor::validatePerceptPreField(const RealisticBallPercept& ball) const
{
  const int centerX = static_cast<int>(ball.positionInImage.x());
  const int centerY = static_cast<int>(ball.positionInImage.y());

  if(centerX >= 0 &&
     centerX < theImage.width &&
     centerY > (theFieldBoundary.isValid ? std::max(0, theFieldBoundary.getBoundaryY(centerX)) : 0) &&
     centerY < theImage.height)
  {
    if(theCameraInfo.camera == CameraInfo::Camera::lower)
    {
      int clippedBottom = centerY;
      theBodyContour.clipBottom(centerX, clippedBottom, theImage.height);
      if(centerY > clippedBottom)
      {
        return false;
      }
    }

    const int r = static_cast<int>(ball.radiusInImage);
    const int rDiagonal = r * 7 / 10; // approximately sin(45deg) = cos(45deg)
    int greenCounter = 0;
    int checked = 1;
    if(theColorTable[theImage[centerY][centerX]].is(ColorClasses::green))
    {
      greenCounter++;
    }
    for(int x = centerX - 1, y = centerY - 1; x >= std::max(0, centerX - rDiagonal) && y >= std::max(0, centerY - rDiagonal); x--, y--)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX + 1, y = centerY - 1; x < std::min(theImage.width, centerX + 1 + rDiagonal) && y >= std::max(0, centerY - rDiagonal); x++, y--)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX + 1, y = centerY + 1; x < std::min(theImage.width, centerX + 1 + rDiagonal) && y < std::min(theImage.height, centerY + 1 + rDiagonal); x++, y++)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX - 1, y = centerY + 1; x >= std::max(0, centerX - rDiagonal) && y < std::min(theImage.height, centerY + 1 + rDiagonal); x--, y++)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX, y = centerY - 1; y >= std::max(0, centerY - r); y--)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX, y = centerY + 1; y < std::min(theImage.height, centerY + 1 + r); y++)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX - 1, y = centerY; x >= std::max(0, centerX - r); x--)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }
    for(int x = centerX + 1, y = centerY; x < std::min(theImage.width, centerX + 1 + r); x++)
    {
      checked++;
      if(theColorTable[theImage[y][x]].is(ColorClasses::green))
      {
        greenCounter++;
      }
    }

    const float greenPart = (float)greenCounter / (float)checked;
    return greenPart < maxGreenThreshold || greenPart > minGreenThreshold;
  }

  return false;
}

void RealisticBallPerceptor::findEdges()
{
  const unsigned short horizon = std::min(static_cast<unsigned short>(std::max(2.f, theImageCoordinateSystem.origin.y())), static_cast<unsigned short>(theCameraInfo.height - 3));
  yStart = static_cast<unsigned short>(theImage.height);

  if(theCameraInfo.camera == CameraInfo::Camera::upper && theFieldBoundary.isValid)
  {
    for(int x = 0; x < theImage.width; x++)
    {
      const unsigned short y = static_cast<unsigned short>(std::max<int>(horizon, theFieldBoundary.getBoundaryY(x)));
      minY[x] = y;
      if(y < yStart)
      {
        yStart = y;
      }
    }
    for(int x = 0; x < theImage.width; x++)
    {
      minY[x] = static_cast<unsigned short>(std::max(0, minY[x] - yStart));
      maxY[x] = static_cast<unsigned short>(std::max(0, theImage.height - yStart));
    }
  }
  else
  {
    yStart = horizon;
    if(theCameraInfo.camera == CameraInfo::Camera::lower)
    {
      for(int x = 0; x < theImage.width; x++)
      {
        minY[x] = 0;
        int y = theImage.height;
        theBodyContour.clipBottom(x, y, theImage.height);
        maxY[x] = static_cast<unsigned short>(std::max(0, y - yStart));
      }
    }
    else
    {
      for(int x = 0; x < theImage.width; x++)
      {
        minY[x] = 0;
        maxY[x] = static_cast<unsigned short>(theImage.height - yStart);
      }
    }
  }

  for(int i = 0; i < Image::maxResolutionHeight; i++)
  {
    points[i].clear();
  }
  anchors.clear();
  width = static_cast<unsigned short>(theImage.width);
  height = static_cast<unsigned short>(theImage.height);
  rows = static_cast<unsigned short>(theImage.height - yStart);
  count = 0;

  edgels.clear();
  if(edgeDetectMode == EdgeDetectMode::edgeDrawing)
  {
    Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Y, yStart);
    Sobel::sobelSSE(image1D, sobelImage);
    buildSimpleGradientDirMapFirstChannel();
    if(cbChannel)
    {
      Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Cb, yStart);
      Sobel::sobelSSE(image1D, sobelImage);
      buildSimpleGradientDirMapAddChannel();
    }
    if(crChannel)
    {
      Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Cr, yStart);
      Sobel::sobelSSE(image1D, sobelImage);
      buildSimpleGradientDirMapAddChannel();
    }

    edgeDrawing();
  }
  else
  {
    Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Y, yStart);
    Sobel::sobelSSE(image1D, sobelImage);
    if(edgeDetectMode == EdgeDetectMode::sobel)
    {
      sobel();
      if(cbChannel)
      {
        Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Cb, yStart);
        Sobel::sobelSSE(image1D, sobelImage);
        sobel();
      }
      if(crChannel)
      {
        Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Cr, yStart);
        Sobel::sobelSSE(image1D, sobelImage);
        sobel();
      }
    }
    else
    {
      buildGradientDirMapFirstChannel();
      if(cbChannel)
      {
        Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Cb, yStart);
        Sobel::sobelSSE(image1D, sobelImage);
        buildGradientDirMapAddChannel();
      }
      if(crChannel)
      {
        Sobel::extractSingleChannelSSE(theImage, image1D, Sobel::Channel::Cr, yStart);
        Sobel::sobelSSE(image1D, sobelImage);
        buildGradientDirMapAddChannel();
      }
      sobelNMS();
    }
  }

  // Collect points
  for(const Point& p : edgels)
  {
    points[p.y].push_back(p.x);
  }
  count = static_cast<unsigned int>(edgels.size());
}

void RealisticBallPerceptor::buildGradientDirMapFirstChannel()
{
  for(unsigned short x = 0; x < width; x++)
  {
    const unsigned short yMax = std::min(maxY[x], rows);
    const unsigned short yMin = std::min(minY[x], yMax);
    for(unsigned short y = 0; y < yMin; y++)
    {
      gdMap(x, y).gradient = 0;
    }
    for(unsigned short y = yMin; y < yMax; y++)
    {
      gdMap(x, y).fromSobelPx(sobelImage(x, y + yStart), gradientThreshold);
    }
    for(unsigned short y = yMax; y < rows; y++)
    {
      gdMap(x, y).gradient = 0;
    }
  }
}

void RealisticBallPerceptor::buildSimpleGradientDirMapFirstChannel()
{
  for(unsigned short x = 0; x < width; x++)
  {
    const unsigned short yMax = std::min(maxY[x], rows);
    const unsigned short yMin = std::min(minY[x], yMax);
    for(unsigned short y = 0; y < yMin; y++)
    {
      gdMap(x, y).gradient = 0;
    }
    for(unsigned short y = yMin; y < yMax; y++)
    {
      gdMap(x, y).fromSobelPxSimple(sobelImage(x, y + yStart), gradientThreshold);
    }
    for(unsigned short y = yMax; y < rows; y++)
    {
      gdMap(x, y).gradient = 0;
    }
  }
}

void RealisticBallPerceptor::buildGradientDirMapAddChannel()
{
  for(unsigned short x = 0; x < width; x++)
  {
    const unsigned short yMax = std::min(maxY[x], rows);
    const unsigned short yMin = std::min(minY[x], yMax);
    for(unsigned short y = 0; y < yMin; y++)
    {
      gdMap(x, y).gradient = 0;
    }
    for(unsigned short y = yMin; y < yMax; y++)
    {
      gdMap(x, y).addSobelPx(sobelImage(x, y + yStart), gradientThreshold);
    }
    for(unsigned short y = yMax; y < rows; y++)
    {
      gdMap(x, y).gradient = 0;
    }
  }
}

void RealisticBallPerceptor::buildSimpleGradientDirMapAddChannel()
{
  for(unsigned short x = 0; x < width; x++)
  {
    const unsigned short yMax = std::min(maxY[x], rows);
    const unsigned short yMin = std::min(minY[x], yMax);
    for(unsigned short y = 0; y < yMin; y++)
    {
      gdMap(x, y).gradient = 0;
    }
    for(unsigned short y = yMin; y < yMax; y++)
    {
      gdMap(x, y).addSobelPxSimple(sobelImage(x, y + yStart), gradientThreshold);
    }
    for(unsigned short y = yMax; y < rows; y++)
    {
      gdMap(x, y).gradient = 0;
    }
  }
}

void RealisticBallPerceptor::sobel()
{
  for(unsigned short y = 0; y < rows; y++)
  {
    const Sobel::SobelPixel* row = sobelImage[y + yStart];
    for(unsigned short x = 0; x < width; x++)
    {
      if(y >= minY[x] && y < maxY[x])
      {
        const Sobel::SobelPixel p = row[x];
        if(std::abs(p.x) + std::abs(p.y) >= gradientThreshold)
        {
          edgels.insert(Point(x, y));
        }
      }
    }
  }
}

void RealisticBallPerceptor::sobelNMS()
{
  for(unsigned short y = 1; y < rows - 1; y++)
  {
    GradientDirMap::Pixel* up = gdMap[y - 1];
    GradientDirMap::Pixel* cur = gdMap[y];
    GradientDirMap::Pixel* down = gdMap[y + 1];
    for(unsigned short x = 1; x < width - 1; x++, up++, cur++, down++)
    {
      const unsigned char grad = cur[1].gradient;
      if(grad > 0)
      {
        switch(cur[1].dir)
        {
          case GradientDirMap::Pixel::Dir::HORIZONTAL:
            if(up[1].gradient < grad && down[1].gradient < grad)
            {
              edgels.insert(Point(x, y));
            }
            break;
          case GradientDirMap::Pixel::Dir::DIAGONALUP:
            if(up[0].gradient < grad && down[2].gradient < grad)
            {
              edgels.insert(Point(x, y));
            }
            break;
          case GradientDirMap::Pixel::Dir::VERTICAL:
            if(cur[0].gradient < grad && cur[2].gradient < grad)
            {
              edgels.insert(Point(x, y));
            }
            break;
          case GradientDirMap::Pixel::Dir::DIAGONALDOWN:
            if(up[2].gradient < grad && down[0].gradient < grad)
            {
              edgels.insert(Point(x, y));
            }
            break;
          default:
            ASSERT(false);
        }
      }
    }
  }
}

void RealisticBallPerceptor::edgeDrawing()
{
  // Select anchors
  for(unsigned short y = 0; y < rows; y += edAnchorGridSize)
  {
    const GradientDirMap::Pixel* row = gdMap[y];
    for(unsigned short x = 0; x < width; x += edAnchorGridSize)
    {
      const GradientDirMap::Pixel& val = row[x];
      if(val.gradient > 0)
      {
        if(val.dir == GradientDirMap::Pixel::Dir::HORIZONTAL)
        {
          if((y == 0 || val.gradient - gdMap(x, y - 1).gradient >= edAnchorThreshold) && (y == rows - 1 || val.gradient - gdMap(x, y + 1).gradient >= edAnchorThreshold))
          {
            anchors.emplace_back(x, y);
          }
        }
        else
        {
          if((x == 0 || val.gradient - row[x - 1].gradient >= edAnchorThreshold) && (x == width - 1 || val.gradient - row[x + 1].gradient >= edAnchorThreshold))
          {
            anchors.emplace_back(x, y);
          }
        }
      }
    }
  }

  // Draw edges
  for(const Point& anchor : anchors)
  {
    edgels.insert(anchor);
    unsigned int l = 1;
    if(gdMap(anchor.x, anchor.y).dir == GradientDirMap::Pixel::Dir::HORIZONTAL)
    {
      edGoLeft(anchor, l);
      edGoRight(anchor, l);
    }
    else
    {
      edGoUp(anchor, l);
      edGoDown(anchor, l);
    }
  }
}

void RealisticBallPerceptor::edGoLeft(const Point& p, unsigned int& length)
{
  if(p.x > 0)
  {
    Point p2(p.x - 1, p.y);
    const GradientDirMap::Pixel& straight = gdMap(p2);
    const GradientDirMap::Pixel& up = gdMap(p2.x, p2.y - 1);
    const GradientDirMap::Pixel& down = gdMap(p2.x, p2.y + 1);
    // Look at 3 neighbors to the left & pick the one with the max. gradient value
    if(p2.y > 0 && up.gradient > 0 && up.gradient > straight.gradient && (p2.y == rows - 1 || up.gradient > down.gradient))
    {
      // Up-Left
      p2.y--;
    }
    else if(p2.y < rows - 1 && down.gradient > 0 && down.gradient > straight.gradient && (p2.y == 0 || down.gradient > up.gradient))
    {
      // Down-Left
      p2.y++;
    }
    else if(straight.gradient <= 0)
    {
      return;
    }

    if(edgels.find(p2) == edgels.end())
    {
      edgels.insert(p2);
      length++;
      if(gdMap(p2).dir == GradientDirMap::Pixel::Dir::HORIZONTAL)
      {
        edGoLeft(p2, length);
      }
      else
      {
        edGoUp(p2, length);
        edGoDown(p2, length);
      }
      if(length < 3)
      {
        edgels.erase(p2);
        length--;
      }
    }
  }
}

void RealisticBallPerceptor::edGoRight(const Point& p, unsigned int& length)
{
  if(p.x < width - 1)
  {
    Point p2(p.x + 1, p.y);
    const GradientDirMap::Pixel& straight = gdMap(p2);
    const GradientDirMap::Pixel& up = gdMap(p2.x, p2.y - 1);
    const GradientDirMap::Pixel& down = gdMap(p2.x, p2.y + 1);
    // Look at 3 neighbors to the right & pick the one with the max. gradient value
    if(p2.y > 0 && up.gradient > 0 && up.gradient > straight.gradient && (p2.y == rows - 1 || up.gradient > down.gradient))
    {
      // Up-Right
      p2.y--;
    }
    else if(p2.y < rows - 1 && down.gradient > 0 && down.gradient > straight.gradient && (p2.y == 0 || down.gradient > up.gradient))
    {
      // Down-Right
      p2.y++;
    }
    else if(straight.gradient <= 0)
    {
      return;
    }

    if(edgels.find(p2) == edgels.end())
    {
      edgels.insert(p2);
      length++;
      if(gdMap(p2).dir == GradientDirMap::Pixel::Dir::HORIZONTAL)
      {
        edGoRight(p2, length);
      }
      else
      {
        edGoUp(p2, length);
        edGoDown(p2, length);
      }
      if(length < 3)
      {
        edgels.erase(p2);
        length--;
      }
    }
  }
}

void RealisticBallPerceptor::edGoUp(const Point& p, unsigned int& length)
{
  if(p.y > 0)
  {
    Point p2(p.x, p.y - 1);
    const GradientDirMap::Pixel& straight = gdMap(p2);
    const GradientDirMap::Pixel& left = gdMap(p2.x - 1, p2.y);
    const GradientDirMap::Pixel& right = gdMap(p2.x + 1, p2.y);
    // Look at 3 neighbors to the top & pick the one with the max. gradient value
    if(p2.x > 0 && left.gradient > 0 && left.gradient > straight.gradient && (p2.x == width - 1 || left.gradient > right.gradient))
    {
      // Up-Left
      p2.x--;
    }
    else if(p2.x < width - 1 && right.gradient > 0 && right.gradient > straight.gradient && (p2.x == 0 || right.gradient > left.gradient))
    {
      // Up-Right
      p2.x++;
    }
    else if(straight.gradient <= 0)
    {
      return;
    }

    if(edgels.find(p2) == edgels.end())
    {
      edgels.insert(p2);
      length++;
      if(gdMap(p2).dir == GradientDirMap::Pixel::Dir::HORIZONTAL)
      {
        edGoLeft(p2, length);
        edGoRight(p2, length);
      }
      else
      {
        edGoUp(p2, length);
      }
      if(length < 3)
      {
        edgels.erase(p2);
        length--;
      }
    }
  }
}

void RealisticBallPerceptor::edGoDown(const Point& p, unsigned int& length)
{
  if(p.y < rows - 1)
  {
    Point p2(p.x, p.y + 1);
    const GradientDirMap::Pixel& straight = gdMap(p2);
    const GradientDirMap::Pixel& left = gdMap(p2.x - 1, p2.y);
    const GradientDirMap::Pixel& right = gdMap(p2.x + 1, p2.y);
    // Look at 3 neighbors to the bottom & pick the one with the max. gradient value
    if(p2.x > 0 && left.gradient > 0 && left.gradient > straight.gradient && (p2.x == width - 1 || left.gradient > right.gradient))
    {
      // Down-Left
      p2.x--;
    }
    else if(p2.x < width - 1 && right.gradient > 0 && right.gradient > straight.gradient && (p2.x == 0 || right.gradient > left.gradient))
    {
      // Down-Right
      p2.x++;
    }
    else if(straight.gradient <= 0)
    {
      return;
    }

    if(edgels.find(p2) == edgels.end())
    {
      edgels.insert(p2);
      length++;
      if(gdMap(p2).dir == GradientDirMap::Pixel::Dir::HORIZONTAL)
      {
        edGoLeft(p2, length);
        edGoRight(p2, length);
      }
      else
      {
        edGoDown(p2, length);
      }
      if(length < 3)
      {
        edgels.erase(p2);
        length--;
      }
    }
  }
}

RealisticBallPerceptor::RealisticBallPerceptor() :
  image1D(Image::maxResolutionWidth, Image::maxResolutionHeight), sobelImage(Image::maxResolutionWidth, Image::maxResolutionHeight), 
  gdMap(Image::maxResolutionWidth, Image::maxResolutionHeight)
{
  cells.fill(nullptr);
}

RealisticBallPerceptor::Circle::Circle(short x1, short y1, short x2, short y2, short x3, short y3) :
  center(Vector2f::Zero()), radius(nanf("")), accum(1)
{
  if((x1 != x3 || y1 != y3) && (x2 != x3 || y2 != y3) && (x1 != x2 || y1 != y2) && (x1 != x3 || x2 != x3))
  {
    float p1x, p1y, p2x, p2y, p3x, p3y;

    if(x1 == x3 || y1 == y3)
    {
      if(x1 == x2)
      {
        p1x = (float)x2;
        p1y = (float)y2;
        p2x = (float)x3;
        p2y = (float)y3;
        p3x = (float)x1;
        p3y = (float)y1;
      }
      else
      {
        p1x = (float)x3;
        p1y = (float)y3;
        p2x = (float)x2;
        p2y = (float)y2;
        p3x = (float)x1;
        p3y = (float)y1;
      }
    }
    else
    {
      if(x1 == x2)
      {
        p1x = (float)x1;
        p1y = (float)y1;
        p2x = (float)x3;
        p2y = (float)y3;
        p3x = (float)x2;
        p3y = (float)y2;
      }
      else
      {
        p1x = (float)x3;
        p1y = (float)y3;
        p2x = (float)x1;
        p2y = (float)y1;
        p3x = (float)x2;
        p3y = (float)y2;
      }
    }

    float ma, mb;
    if(p1x != p2x)
    {
      ma = (p2y - p1y) / (p2x - p1x);
    }
    else
    {
      ma = 1000000;
    }
    if(p2x != p3x)
    {
      mb = (p3y - p2y) / (p3x - p2x);
    }
    else
    {
      mb = 1000000;
    }

    if(ma != 0 || mb != 0)
    {
      center.x() = (ma * mb * (p1y - p3y) + mb * (p1x + p2x) - ma * (p2x + p3x)) / (2 * (mb - ma));
      if(ma != 0)
      {
        center.y() = -(center.x() - (p1x + p2x) / 2) / ma + (p1y + p2y) / 2;
      }
      else
      {
        center.y() = -(center.x() - (p2x + p3x) / 2) / mb + (p2y + p3y) / 2;
      }
      radius = (center - Vector2f((float)x1, (float)y1)).norm();
    }
  }
}

/**
* Fast randomized hough transform algorithm. The algorithm starts at srcImage.yStart.
*
* @param srcImage The source image.
* @param destImage The destination image.
*/
void RealisticBallPerceptor::fastRHT(std::vector<Circle>& circles, unsigned char windowPadding, unsigned short expectedPixelsPerCircle,
                                     float minRadius, float maxRadius, unsigned int circleThreshold,
                                     float mergeRadius, float mergeDistanceSquared)
{
  const int probability = RAND_MAX / expectedPixelsPerCircle;

  // Iterate over points
  for(unsigned short y = 0; y < rows; y++)
  {
    for(const unsigned short& x : points[y])
    {
      // Choose seed points randomly
      if(rand() < probability)
      {
        fastHTPoint(x, y, circles, windowPadding, minRadius, maxRadius, circleThreshold, mergeRadius, mergeDistanceSquared);
      }
    }
  }
}

void RealisticBallPerceptor::fastHT(std::vector<Circle>& circles, unsigned char windowPadding, float minRadius, float maxRadius,
                                    unsigned int circleThreshold, float mergeRadius, float mergeDistanceSquared)
{
  // Iterate over anchors
  for(const Point& anchor : anchors)
  {
    fastHTPoint(anchor.x, anchor.y, circles, windowPadding, minRadius, maxRadius, circleThreshold, mergeRadius, mergeDistanceSquared);
  }
}

void RealisticBallPerceptor::fastHTPoint(short x, short y, std::vector<Circle>& circles, unsigned char windowPadding,
                                         float minRadius, float maxRadius, unsigned int circleThreshold,
                                         float mergeRadius, float mergeDistanceSquared)
{
  if(!theBodyContour.isValidPoint(Vector2i(x, y + yStart)))
  {
    return;
  }
  unsigned int maxAccum = 0;
  std::vector<Circle> candidateCircles;

  // Accumulate points on the same circle as the seed point by looking at
  // points within a window aroud the point
  RECTANGLE("module:RealisticBallPerceptor:Window", std::max(0, x - windowRadius),
            yStart + std::max(0, y - windowRadius),
            std::min<int>(width, x + windowRadius),
            yStart + std::min<int>(rows, y + windowRadius),
            1, Drawings::dashedPen, ColorRGBA::gray);
  for(unsigned short windowY = static_cast<unsigned short>(std::max(0, y - windowRadius)); windowY < std::min<unsigned short>(rows, y + windowRadius); windowY++)
  {
    for(const unsigned short& windowX : points[windowY])
    {
      if(windowX < std::max(0, x - windowRadius) || (windowY == y && windowX == x))
      {
        continue;
      }
      if(windowX >= std::min<unsigned short>(width, x + windowRadius))
      {
        break;
      }

      const unsigned char distance = static_cast<unsigned char>(std::round(std::hypot(static_cast<float>(windowX) - x, static_cast<float>(windowY) - y)));
      if(distance >= windowPadding)
      {
        if(cells[distance] == nullptr)
        {
          cells[distance] = new Vector2s(static_cast<short>(windowX), static_cast<short>(windowY));
        }
        else
        {
          // We found two edge points with the same distance from the seed
          // point.
          // Compute the center and radius of the circle on which the points
          // may lie and increment the accumulator of this circle.
          Circle candidate(x, y, static_cast<short>(windowX), static_cast<short>(windowY), cells[distance]->x(), cells[distance]->y());
          candidate.center.y() += yStart;

          if(!std::isnan(candidate.radius) && !std::isnan(candidate.center.x()) && !std::isnan(candidate.center.y()))
          {
            if(candidate.radius >= minRadius && candidate.radius <= maxRadius)
            {
              for(Circle& circle : candidateCircles)
              {
                if(std::abs(circle.radius - candidate.radius) <= mergeRadius &&
                   (circle.center - candidate.center).squaredNorm() <= mergeDistanceSquared)
                {
                  circle.radius = (circle.radius + candidate.radius) / 2;
                  circle.center = (circle.center + candidate.center) / 2;
                  maxAccum = std::max(maxAccum, ++circle.accum);
                  goto candidateExists;
                }
              }
              candidateCircles.push_back(candidate);
candidateExists:
              ;
            }
          }
        }
      }
    }
  }

  if(maxAccum >= circleThreshold)
  {
    for(const Circle& candidate : candidateCircles)
    {
      if(candidate.accum == maxAccum)
      {
        CIRCLE("module:RealisticBallPerceptor:Circle", candidate.center.x(), candidate.center.y(), candidate.radius, 1,
               Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
        for(Circle& circle : circles)
        {
          if(std::abs(circle.radius - candidate.radius) <= mergeRadius &&
             (circle.center - candidate.center).squaredNorm() <= mergeDistanceSquared)
          {
            circle.radius = (circle.radius + candidate.radius) / 2.f;
            circle.center = (circle.center + candidate.center) / 2.f;
            circle.accum += candidate.accum;
            goto circleExists;
          }
        }
        circles.push_back(candidate);
circleExists:
        ;
      }
    }
  }

  for(Vector2s*& c : cells)
  {
    if(c != nullptr)
    {
      delete c;
      c = nullptr;
    }
  }
}
