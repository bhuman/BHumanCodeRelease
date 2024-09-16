/**
* @file CenterCircleWithLinePerceptor.cpp
*
* Implementation of a module that tries to find the combination of
* the center circle and the halfway line crossing the circle.
* in the current field percepts. This is the combination
* that this module is looking for:
*
*                   _____
*                  *     *      (ugly, I know)
*                 /       \
*    ------------+----+----+------------
*                 \       /
*                  *     *
*                   -----
*
* @author Tim Laue
*/

// TODO:
// - Implement intersection of line and circle
// - More detailed checks of line
// - Consider rejecting vertical lines?

#include "CenterCircleWithLinePerceptor.h"
#include "Math/BHMath.h"
#include <algorithm>

CenterCircleWithLinePerceptor::CenterCircleWithLinePerceptor()
{
  timeWhenCenterCircleLastSeen = 0;
}

void CenterCircleWithLinePerceptor::update(CenterCircleWithLine& centerCircleWithLine)
{
  centerCircleWithLine.isValid = false;
  // Check for backwards "jump" in log file:
  if(theFrameInfo.time < timeWhenCenterCircleLastSeen)
    timeWhenCenterCircleLastSeen = 0;
  // Buffer a new center circle:
  if(theCirclePercept.wasSeen)
  {
    centerCirclePosition = theCirclePercept.pos;
    centerCircleCovariance = theCirclePercept.cov;
    timeWhenCenterCircleLastSeen = theFrameInfo.time;
  }
  // If there is no new center circle, try to update the buffered one by odometry:
  else if(theFrameInfo.getTimeSince(timeWhenCenterCircleLastSeen) < bufferTimeCenterCircle)
  {
    const Pose2f odometryOffset = theOdometer.odometryOffset.inverse();
    centerCirclePosition = odometryOffset * centerCirclePosition;
  }
  // There is no center circle that we can currently use:
  else
  {
    return;
  }
  // There seems to be a center circle available, so let's find the halfway line:
  if(findIntersectingHalfwayLine())
    computeFeature(centerCircleWithLine);
}

bool CenterCircleWithLinePerceptor::findIntersectingHalfwayLine()
{
  // Iterate over all lines and take the first line that fulfills all criteria.
  // As lines are sorted by length, processing is stopped when the first line
  // that would be too short is reached.
  for(const auto& line : theFieldLines.lines)
  {
    if(line.length < minimumLengthOfPerceivedLine) // Line is too short, following lines will be too short, too.
      return false;
    if(std::abs(line.alpha) > maxAbsAngleAlpha) // Line is "too vertical", avoid false positives
      continue;

    float distanceToCenter = 0.f;
    const Geometry::Line geoLine(line.first, line.last - line.first);
    if(line.length > maxExtraCheckingLineLength) // "Long" line, can end not so close to the center
      distanceToCenter = Geometry::getDistanceToLine(geoLine, centerCirclePosition);
    else // Short line, perform stricter check for edge distance to center circle center
      distanceToCenter = Geometry::getDistanceToEdge(geoLine, centerCirclePosition);

    if(distanceToCenter < maxLineDeviationFromCenter)
    {
      halfwayLine = geoLine;
      halfwayLineCov = line.cov;
      return true;
    }
  }
  return false;
}

void CenterCircleWithLinePerceptor::computeFeature(CenterCircleWithLine& centerCircleWithLine)
{
  // Determine the pose:
  Vector2f lineDirection = halfwayLine.direction;
  lineDirection.rotateLeft();
  Pose2f perceivedPose(lineDirection.angle(), centerCirclePosition);
  centerCircleWithLine = perceivedPose;
  // Compute covariance:
  Pose2f computedRobotPose;
  centerCircleWithLine.isValid = true;
  if(centerCircleWithLine.pickMorePlausiblePose(theWorldModelPrediction.robotPose, computedRobotPose))
  {
    const float c = std::cos(computedRobotPose.rotation);
    const float s = std::sin(computedRobotPose.rotation);
    const Matrix2f angleRotationMatrix = (Matrix2f() << c, -s, s, c).finished();
    const Matrix2f covXR = angleRotationMatrix * halfwayLineCov * angleRotationMatrix.transpose();
    const Matrix2f covY = angleRotationMatrix * centerCircleCovariance * angleRotationMatrix.transpose();
    const float xVariance = covXR(0, 0); // Depends on line
    const float yVariance = covY(1, 1);  // Depends on circle center
    const float sqrLineLength = std::max(sqr(2 * theFieldDimensions.centerCircleRadius), halfwayLine.direction.squaredNorm());
    const float angleVariance = sqr(std::atan(std::sqrt (4.f * xVariance / sqrLineLength)));
    centerCircleWithLine.covOfAbsoluteRobotPose << xVariance, 0.f, 0.f, 0.f, yVariance, 0.f, 0.f, 0.f, angleVariance;
  }
  else
  {
    centerCircleWithLine.isValid = false;
  }
}

MAKE_MODULE(CenterCircleWithLinePerceptor);
