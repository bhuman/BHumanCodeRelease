/**
 * @file ObstaclesPerceptor.cpp
 *
 * This file implements a module that detects obstacles in the color-segmented image.
 * This is a more compact rewrite of the RobotsPerceptor by Michel Bartsch.
 * Points below obstacles are found by vertically scanning down from the field boundary.
 * Such a candidate point is the lowest edge between non-field color and field color
 * where enough non-field color came above it. These candidates are processed from
 * bottom to top, merging neighboring candidates on a similar height.
 *
 * @author Thomas Röfer
 */

#include "ObstaclesPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>

MAKE_MODULE(ObstaclesPerceptor, perception)

/**
 * Hue values for the different team colors. The indices are those of the TEAM_
 * constants defined in RoboCupGameControlData.h.
 */
static int jerseyHues[] =
{
  255 * 7 / 8, // blue + cyan
  255 * 2 / 8, // red
  255 * 4 / 8, // yellow
  0, // black (unused)
  0, // white (unused)
  0, // green (unsupported)
  255 * 3 / 8, // orange
  255 * 1 / 8, // purple
  255 * 7 / 16, // brown
  0, // gray (unused)
};

void ObstaclesPerceptor::update(ObstaclesImagePercept& theObstaclesImagePercept)
{
  DECLARE_DEBUG_DRAWING("module:ObstaclesPerceptor:grid", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstaclesPerceptor:spots", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstaclesPerceptor:candidates", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ObstaclesPerceptor:jersey", "drawingOnImage");
  theObstaclesImagePercept.obstacles.clear();

  Vector2f topOnField;
  Vector2f bottomOnField;
  if(theFieldBoundary.isValid
     && Transformation::imageToRobot(Vector2i(theCameraInfo.width / 2, theScanGrid.y.back()), theCameraMatrix, theCameraInfo, topOnField)
     && Transformation::imageToRobot(Vector2i(theCameraInfo.width / 2, theCameraInfo.height), theCameraMatrix, theCameraInfo, bottomOnField))
  {
    STOPWATCH("module:ObstaclesPerceptor:project")
      project();

    // Compute sizes that allow to quickly interpole between them based on a vertical coordinate.
    const float topUnit = Projection::getSizeByDistance(theCameraInfo, 1.f, (Vector3f(topOnField.x(), topOnField.y(), 0.f) - theCameraMatrix.translation).norm());
    const float bottomUnit = Projection::getSizeByDistance(theCameraInfo, 1.f, (Vector3f(bottomOnField.x(), bottomOnField.y(), 0.f) - theCameraMatrix.translation).norm());
    const int topStep = static_cast<int>(yStep * topUnit + 0.5f);
    const int bottomStep = static_cast<int>(yStep * bottomUnit + 0.5f);
    ScanResult* const scanResults = this->scanResults[theCameraInfo.camera];

    // Draw the grid
    COMPLEX_DRAWING("module:ObstaclesPerceptor:grid")
      draw(scanResults, theScanGrid.y.back(), topStep, bottomStep);

    // Find candidate points for obstacles.
    std::vector<ScanResult*> candidates;
    STOPWATCH("module:ObstaclesPerceptor:scan")
      for(int x = 0; x < theCameraInfo.width; x += xStep)
        if(scan(x, theScanGrid.y.back(), topStep, bottomStep, scanResults[x]))
        {
          candidates.push_back(&scanResults[x]);
          CROSS("module:ObstaclesPerceptor:spots", x, scanResults[x].y, 1, 1, Drawings::solidPen, ColorRGBA::orange);
          TIP("module:ObstaclesPerceptor:spots", x, scanResults[x].y, xStep,
              "score:\t" << scanResults[x].score
              << "\ngreenAbove:\t" << scanResults[x].greenAbove);
        }

    // Sort candidates by their y coordinates, lowest in image first.
    STOPWATCH("module:ObstaclesPerceptor:sort")
      std::sort(candidates.begin(), candidates.end(), [](const ScanResult* a, const ScanResult* b) {return a->y > b->y;});

    // Check all candidates and update the percept.
    STOPWATCH("module:ObstaclesPerceptor:check")
      for(ScanResult* candidate : candidates)
        if(static_cast<int>(theObstaclesImagePercept.obstacles.size()) == maxObstaclesToDetect)
          break;
        else if(candidate->y)
          check(scanResults, static_cast<int>(candidate - scanResults), theScanGrid.y.back(), topUnit, bottomUnit, theObstaclesImagePercept.obstacles);
  }
  else if(theCameraInfo.camera == CameraInfo::upper)
    upperCameraMatrix.isValid = false;
}

void ObstaclesPerceptor::project()
{
  ScanResult* scanResults = this->scanResults[theCameraInfo.camera];
  Vector2f leftOnField;
  Vector2f rightOnField;
  Vector2f leftInImage;
  Vector2f rightInImage;
  const CameraInfo& upperCameraInfo = upperImageCoordinateSystem.cameraInfo;
  if(theCameraInfo.camera == CameraInfo::lower && upperCameraMatrix.isValid
     && Transformation::imageToRobot(upperImageCoordinateSystem.toCorrected(Vector2i(0, upperCameraInfo.height)),
                                     upperCameraMatrix, upperCameraInfo, leftOnField)
     && Transformation::imageToRobot(upperImageCoordinateSystem.toCorrected(Vector2i(upperCameraInfo.width, upperCameraInfo.height)),
                                     upperCameraMatrix, upperCameraInfo, rightOnField))
  {
    leftOnField = theOdometer.odometryOffset.inverse() * leftOnField;
    rightOnField = theOdometer.odometryOffset.inverse() * rightOnField;
    if(Transformation::robotToImage(leftOnField, theCameraMatrix, theCameraInfo, leftInImage)
        && Transformation::robotToImage(rightOnField, theCameraMatrix, theCameraInfo, rightInImage))
    {
      const Vector2i left = theImageCoordinateSystem.fromCorrected(leftInImage).cast<int>();
      const Vector2i diff = theImageCoordinateSystem.fromCorrected(rightInImage).cast<int>() - left;
      if(diff.x() > 0)
      {
        const ScanResult* upperResults = this->scanResults[CameraInfo::upper];
        for(int x = 0; x < theCameraInfo.width; x += xStep)
        {
          const int xUpper = (x - left.x()) * upperCameraInfo.width / diff.x();
          const int yLower = left.y() + (x - left.x()) * diff.y() / diff.x();
          if(xUpper >= 0 && xUpper < upperCameraInfo.width && yLower >= 0 && yLower < theCameraInfo.height)
          {
            const ScanResult& result = upperResults[xUpper / xStep * xStep];
            scanResults[x].y = yLower;
            scanResults[x].shortRangeScore = result.shortRangeScore;
            scanResults[x].longRangeScore = result.shortRangeScore;
          }
          else
            scanResults[x].y = -1;
        }
        return; // Successfully projected
      }
    }
  }
  else if(theCameraInfo.camera == CameraInfo::upper)
  {
    upperCameraMatrix = theCameraMatrix;
    upperImageCoordinateSystem = theImageCoordinateSystem;
  }

  // Mark that nothing could be projected.
  for(int x = 0; x < theCameraInfo.width; x += xStep)
    scanResults[x].y = -1;
}

bool ObstaclesPerceptor::scan(const int x, const int top, const int topStep, const int bottomStep, ScanResult& scanResult) const
{
  const int bottom = theBodyContour.getBottom(x, theCameraInfo.height);
  const float grow = static_cast<float>((topStep - bottomStep) * (topStep + bottomStep)) / static_cast<float>(2 * (top - theCameraInfo.height));
  int yStart = std::max(theFieldBoundary.getBoundaryY(x) + yStartBelow, top);

  // Only use the projection if it is lower than the field boundary.
  if(yStart > scanResult.y)
  {
    scanResult.longRangeScore = longRangeThreshold - shortRangeThreshold;
    scanResult.shortRangeScore = std::min(shortRangeThreshold - minHeight, 0);
  }
  else
    yStart = scanResult.y;

  // Determine initial step size
  float step = static_cast<float>(topStep) + static_cast<float>((yStart - top) * (topStep - bottomStep)) / static_cast<float>(top - theCameraInfo.height);

  // Start one step earlier than the projection
  if(yStart == scanResult.y)
    yStart = std::max(0, yStart - static_cast<int>(step));

  scanResult.y = 0;
  scanResult.greenAbove = false;
  bool foundEnoughGreen = false;

  for(int y = yStart; y < bottom; step += grow, y += static_cast<int>(step))
  {
    const FieldColors::Color& color = theECImage.colored[y][x];
    if(1 << color & fieldColor)
    {
      scanResult.shortRangeScore = std::min(0, scanResult.shortRangeScore);
      if(color == FieldColors::field)
      {
        scanResult.longRangeScore -= longRangePenalty;
        if(scanResult.longRangeScore < 0)
        {
          scanResult.longRangeScore = 0;
          foundEnoughGreen = true;
        }
      }
    }
    else
    {
      ++scanResult.shortRangeScore;
      ++scanResult.longRangeScore;
      if(scanResult.shortRangeScore >= shortRangeThreshold && scanResult.longRangeScore >= longRangeThreshold)
      {
        scanResult.y = y;
        scanResult.score = scanResult.longRangeScore;
        scanResult.greenAbove = foundEnoughGreen;
      }
    }
  }
  return scanResult.y != 0;
}

void ObstaclesPerceptor::check(ScanResult* scanResults, const int x, const int top, const float topUnit, const float bottomUnit,
                               std::vector<ObstaclesImagePercept::Obstacle>& obstacles) const
{
  const int y = scanResults[x].y;
  const float unit = topUnit + static_cast<float>(y - top) * (bottomUnit - topUnit) / static_cast<float>(theCameraInfo.height - top);
  const int xTolerance = static_cast<int>(maxXGap * unit + 0.5f);
  const int yTolerance = std::min(std::max(static_cast<int>(IISC::getImageDiameterByLowestPointAndFieldDiameter(
                                           maxYGap, Vector2i(x, y).cast<float>(), theCameraInfo, theCameraMatrix) + 0.5f),
                                           static_cast<int>(yStep * unit + 1.5f)), y - 1);
  const int minWidth = std::max(static_cast<int>(this->minWidth * unit + 0.5f), minWidthInPixel);
  int numOfSupporters = 0;
  int numOfNeighborships = 0;
  int sumOfScores = 0;

  // Expand to the left.
  // Candidate points must have a similar y coordinate and gaps up to a certain width are skipped.
  int left = x;
  for(int xLeft = x - xStep; xLeft >= 0 && xLeft >= left - xTolerance; xLeft -= xStep)
    if(y - yTolerance <= scanResults[xLeft].y)
    {
      ++numOfSupporters;
      sumOfScores += scanResults[xLeft].score;
      if(xLeft == left - xStep)
        ++numOfNeighborships;
      left = xLeft;
    }

  // Expand to the right.
  int right = x;
  for(int xRight = x + xStep; xRight < theCameraInfo.width && xRight <= right + xTolerance; xRight += xStep)
    if(y - yTolerance <= scanResults[xRight].y)
    {
      ++numOfSupporters;
      sumOfScores += scanResults[xRight].score;
      if(xRight == right + xStep)
        ++numOfNeighborships;
      right = xRight;
    }

  const int width = right - left + 1;

  CROSS("module:ObstaclesPerceptor:candidates", x, scanResults[x].y, 1, 1, Drawings::solidPen, ColorRGBA::red);
  TIP("module:ObstaclesPerceptor:candidates", x, scanResults[x].y, 5,
      "width:\t" << minWidth << "\t" << width
      << "\nneighborship:\t" << minWidth * minNeigboredRatio << "\t" << numOfNeighborships * xStep
      << "\nneighborship:\t" << numOfSupporters * minNeigboredRatio << "\t" << numOfNeighborships
      << "\nscore:\t" << minWidth * numOfSupporters * minScoreRatio << "\t" << sumOfScores * width);

  // Obstacles must have a minimum width, a minimum ratio of neighborhood, and a minimum overall score.
  // The score only allows narrow objects to be accepted if they are tall (unlike the ball).
  if(width >= minWidth && numOfNeighborships * xStep >= minWidth * minNeigboredRatio
     && numOfNeighborships >= numOfSupporters * minNeigboredRatio
     && sumOfScores * width >= minWidth * numOfSupporters * minScoreRatio)
  {
    // Scan down to find the lower end of the obstacle.
    int bottom = y;
    for(int yBottom = y + 1; yBottom < theCameraInfo.height && yBottom < y + yTolerance && yBottom - bottom <= maxYMicroGap; ++yBottom)
      if(!(1 << theECImage.colored[yBottom][x] & fieldColor))
        bottom = yBottom;

    // Count how many candidate points have green above them.
    int numOfGreenAbove = 0;
    numOfSupporters = 0;
    for(int x = left; x < right; x += xStep)
      if(scanResults[x].y > 0 && scanResults[x].y <= y)
      {
        numOfGreenAbove += scanResults[x].greenAbove ? 1 : 0;
        ++numOfSupporters;
      }

    // Add obstacle to list of detected obstacles.
    const int ignoreWidth = static_cast<int>(this->ignoreWidth * unit + 0.5f);
    ObstaclesImagePercept::Obstacle obstacle;
    obstacle.fallen = numOfGreenAbove >= numOfSupporters * minGreenAboveRatio;
    obstacle.top = std::max(0, bottom - static_cast<int>((obstacle.fallen ? fallenRobotHeight : uprightRobotHeight) * unit + 0.5f));
    obstacle.bottom = bottom;
    obstacle.left = left;
    obstacle.right = right;
    obstacle.bottomFound = scanResults[x].shortRangeScore < shortRangeThreshold || scanResults[x].longRangeScore < longRangeThreshold;
    obstacles.push_back(obstacle);

    // Delete all candidates in the vicinity of the detected obstacle.
    right = std::min(theCameraInfo.width - 1, right + ignoreWidth);
    for(int x = (std::max(0, left - ignoreWidth) + xStep - 1) / xStep * xStep; x <= right; x += xStep)
      scanResults[x].y = 0;
  }
  else
    scanResults[x].y = 0;
}

void ObstaclesPerceptor::update(ObstaclesFieldPercept& theObstaclesFieldPercept)
{
  theObstaclesFieldPercept.obstacles.clear();
  if(theCameraInfo.camera == CameraInfo::upper)
    incompleteObstacles.clear();

  for(const ObstaclesImagePercept::Obstacle& obstacleInImage : theObstaclesImagePercept.obstacles)
  {
    ObstaclesFieldPercept::Obstacle obstacleOnField;
    if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.left, obstacleInImage.bottom)),
                                       theCameraMatrix, theCameraInfo, obstacleOnField.left)
       && Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2i(obstacleInImage.right, obstacleInImage.bottom)),
                                       theCameraMatrix, theCameraInfo, obstacleOnField.right))
    {
      obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;
      obstacleOnField.fallen = obstacleInImage.fallen;
      obstacleOnField.type = ObstaclesFieldPercept::unknown;

      // Only detect jerseys for robots that are not fallen. Robots without bottom in the image are unused in the lower image.
      if(!obstacleOnField.fallen && (theCameraInfo.camera == CameraInfo::upper || obstacleInImage.bottomFound))
      {
        detectJersey(obstacleInImage, obstacleOnField);

        // If no jersey was detected in the lower image, try to find a matching incomplete robot from the upper image.
        if(theCameraInfo.camera == CameraInfo::lower && obstacleInImage.top == 0 && obstacleOnField.type == ObstaclesFieldPercept::unknown)
        {
          const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
          for(const ObstaclesFieldPercept::Obstacle& incompleteObstacle : incompleteObstacles)
            if(range.isInside((theOdometer.odometryOffset.inverse() * incompleteObstacle.center).angle()))
            {
              obstacleOnField.type = incompleteObstacle.type;
              break;
            }
        }
      }

      // Add obstacle to percept or store it as incomplete obstacle if useful.
      if(obstacleInImage.bottomFound)
        theObstaclesFieldPercept.obstacles.push_back(obstacleOnField);
      else if(obstacleOnField.type != ObstaclesFieldPercept::unknown && theCameraInfo.camera == CameraInfo::upper)
        incompleteObstacles.push_back(obstacleOnField);
    }
  }
}

void ObstaclesPerceptor::detectJersey(const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField) const
{
  Vector2f lowerInImage;
  Vector2f upperInImage;
  float distance = obstacleOnField.center.norm() + theRobotDimensions.footLength * 0.5f;

  // If the lower end was not in the image, guess a distance from the width. This can be very wrong.
  if(!obstacleInImage.bottomFound)
  {
    const float estimatedDistance = Projection::getDistanceBySize(theCameraInfo, assumedWidth,
                                                                  static_cast<float>(obstacleInImage.right - obstacleInImage.left));
    const float xFactor = theCameraInfo.focalLengthInv;
    const float yFactor = theCameraInfo.focalLengthHeightInv;
    const Vector3f toCenter(1.f, (theCameraInfo.opticalCenter.x() - (obstacleInImage.left + obstacleInImage.right) / 2) * xFactor,
                                  (theCameraInfo.opticalCenter.y() - obstacleInImage.bottom) * yFactor);
    const Vector3f centerInWorld = theCameraMatrix.rotation * toCenter.normalized(estimatedDistance);
    distance = std::min(distance, Vector2f(centerInWorld.x(), centerInWorld.y()).norm());
  }

  // Determine jersey region.
  const Vector2f centerOnField = obstacleOnField.center.normalized(distance);
  if(Transformation::robotToImage(Vector3f(centerOnField.x(), centerOnField.y(), jerseyYRange.min),
                                  theCameraMatrix, theCameraInfo, lowerInImage)
     && Transformation::robotToImage(Vector3f(centerOnField.x(), centerOnField.y(), jerseyYRange.max),
                                     theCameraMatrix, theCameraInfo, upperInImage))
  {
    lowerInImage = theImageCoordinateSystem.fromCorrected(lowerInImage);
    if(lowerInImage.y() >= jerseyYSamples)
    {
      upperInImage = theImageCoordinateSystem.fromCorrected(upperInImage);
      if(upperInImage.y() < 0)
        upperInImage = lowerInImage + (upperInImage - lowerInImage) * lowerInImage.y() / (lowerInImage.y() - upperInImage.y());
      if(lowerInImage.y() > static_cast<float>(theCameraInfo.height - 1))
        lowerInImage = upperInImage + (lowerInImage - upperInImage) * (static_cast<float>(theCameraInfo.height - 1) - upperInImage.y()) / (lowerInImage.y() - upperInImage.y());

      const int width = obstacleInImage.right - obstacleInImage.left + 1;
      const int ySteps = std::min(jerseyYSamples, static_cast<int>(lowerInImage.y() - upperInImage.y()));
      const float xStep = std::max(1.f, static_cast<float>(width) / static_cast<float>(jerseyXSamples));
      const float yStep = (lowerInImage.y() - upperInImage.y()) / static_cast<float>(ySteps);
      const float xyStep = (lowerInImage.x() - upperInImage.x()) / static_cast<float>(ySteps);

      // If gray is involved, the maximum brightness is determined, to determine gray ralative to it.
      unsigned char maxBrightness = 0;
      if(theOwnTeamInfo.teamColor == TEAM_GRAY || theOpponentTeamInfo.teamColor == TEAM_GRAY)
      {
        Vector2f centerInImage = (Vector2f(static_cast<float>(obstacleInImage.left + obstacleInImage.right) * 0.5f, static_cast<float>(obstacleInImage.bottom)) * (1.f - whiteScanHeightRatio)
                                 + lowerInImage * whiteScanHeightRatio);
        float left = std::max(centerInImage.x() - static_cast<float>(width) * 0.5f, 0.f);
        float right = std::min(centerInImage.x() + static_cast<float>(width) * 0.5f, static_cast<float>(theCameraInfo.width) - 0.5f);
        for(float x = left; x < right; x += xStep)
          maxBrightness = std::max(theECImage.grayscaled[static_cast<int>(centerInImage.y())][static_cast<int>(x)], maxBrightness);
      }

      // Get pixel classifier
      const std::function<bool(int, int)>& isOwn = getPixelClassifier(theOwnTeamInfo.teamColor, theOpponentTeamInfo.teamColor, maxBrightness);
      const std::function<bool(int, int)>& isOpponent = getPixelClassifier(theOpponentTeamInfo.teamColor, theOwnTeamInfo.teamColor, maxBrightness);

      int ownPixels = 0;
      int opponentPixels = 0;
      float left = std::max(upperInImage.x() - static_cast<float>(width) * 0.5f, 0.f);
      float right = std::min(upperInImage.x() + static_cast<float>(width) * 0.5f, static_cast<float>(theCameraInfo.width) - 0.5f);

      for(float y = upperInImage.y(); y < lowerInImage.y();
          y += yStep, left = std::max(left + xyStep, 0.f), right = std::min(right + xyStep, static_cast<float>(theCameraInfo.width)))
        for(float x = left; x < right; x += xStep)
          if(isOwn(static_cast<int>(x), static_cast<int>(y)))
            ++ownPixels;
          else if(isOpponent(static_cast<int>(x), static_cast<int>(y)))
            ++opponentPixels;

      if((ownPixels > minJerseyPixels || opponentPixels > minJerseyPixels)
         && (ownPixels > (ownPixels + opponentPixels) * minJerseyRatio
             || opponentPixels > (ownPixels + opponentPixels) * minJerseyRatio))
        obstacleOnField.type = ownPixels > opponentPixels ? ObstaclesFieldPercept::ownPlayer : ObstaclesFieldPercept::opponentPlayer;
    }
  }
}

std::function<bool(int, int)> ObstaclesPerceptor::getPixelClassifier(const int teamColor, const int otherColor, const int maxBrightness) const
{
  const int teamHue = jerseyHues[teamColor];
  const int otherHue = jerseyHues[otherColor];
  const Rangei grayRange = Rangei(static_cast<int>(maxBrightness * this->grayRange.min),
                                  static_cast<int>(maxBrightness * this->grayRange.max));

  // If gray is involved, different brightnesses must be distinguished.
  if(teamColor == TEAM_GRAY)
    return [this, grayRange](const int x, const int y)
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      return color != FieldColors::none && color != FieldColors::field && grayRange.isInside(theECImage.grayscaled[y][x]);
    };
  else if(teamColor == TEAM_BLACK && otherColor == TEAM_GRAY)
    return [this, grayRange](const int x, const int y)
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      return color != FieldColors::none && color != FieldColors::field && theECImage.grayscaled[y][x] < grayRange.min;
    };
  else if(teamColor == TEAM_WHITE && otherColor == TEAM_GRAY)
    return [this, grayRange](const int x, const int y)
    {
      const FieldColors::Color color = theECImage.colored[y][x];
      return color != FieldColors::none && color != FieldColors::field && theECImage.grayscaled[y][x] > grayRange.max;
    };

  // Black and white can be determined directly
  else if(teamColor == TEAM_BLACK)
    return [this](const int x, const int y) {return theECImage.colored[y][x] == FieldColors::black;};
  else if(teamColor == TEAM_WHITE)
    return [this](const int x, const int y) {return theECImage.colored[y][x] == FieldColors::white;};

  // This team uses a color, the other team does not.
  else if(otherColor == TEAM_WHITE || otherColor == TEAM_BLACK || otherColor == TEAM_GRAY)
    return [this, teamHue](const int x, const int y)
    {
      return theECImage.colored[y][x] == FieldColors::none
             && std::abs(static_cast<char>(theECImage.hued[y][x] - teamHue)) <= hueSimilarityThreshold;
    };

  // Both teams use colors, use the closer one.
  else
    return [this, teamHue, otherHue](const int x, const int y)
    {
      if(theECImage.colored[y][x] == FieldColors::none)
      {
        const int hue = theECImage.hued[y][x];
        // Casting to char should resolve the wraparound cases.
        return std::abs(static_cast<char>(hue - teamHue)) < std::abs(static_cast<char>(hue - otherHue));
      }
      else
        return false;
    };
}

void ObstaclesPerceptor::draw(const ScanResult* scanResults, const int top, const int topStep, const int bottomStep) const
{
  for(int x = 0; x < theCameraInfo.width; x += xStep)
  {
    const int bottom = theBodyContour.getBottom(x, theCameraInfo.height);
    const float grow = static_cast<float>((topStep - bottomStep) * (topStep + bottomStep)) / static_cast<float>(2 * (top - theCameraInfo.height));

    int yStart = std::max(theFieldBoundary.getBoundaryY(x) + yStartBelow, top);
    if(yStart == 0 && scanResults[x].y > -1)
      yStart = scanResults[x].y;

    float step = static_cast<float>(topStep) + static_cast<float>((yStart - top) * (topStep - bottomStep)) / static_cast<float>(top - theCameraInfo.height);
    if(scanResults[x].y > -1)
      yStart = std::max(0, yStart - static_cast<int>(step));

    for(int y = yStart; y < bottom; step += grow, y += static_cast<int>(step))
      LINE("module:ObstaclesPerceptor:grid", x, y, x, y, 0, Drawings::solidPen, ColorRGBA::yellow);
  }
}
