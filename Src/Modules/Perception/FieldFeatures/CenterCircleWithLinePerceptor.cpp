/**
* @file CenterCircleWithLinePerceptor.cpp
*
* Implementation of a module that tries to find the combination of
* the center circle and the center line crossing the circle.
* in the current field percepts. This is the combination
* that this modul is looking for:
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
  // There seems to be a center circle available, so let's find the center line:
  if(findIntersectingCenterLine())
  {
    computePose(centerCircleWithLine);
    centerCircleWithLine.isValid = true;
  }
}

bool CenterCircleWithLinePerceptor::findIntersectingCenterLine()
{
  // Iterate over all lines and take the first line that fulfills all criteria.
  // As lines are sorted by length, processing is stopped when the first line
  // that would be too short is reached.
  for(const auto& line : theFieldLines.lines)
  {
    if(line.length < minimumLengthOfPerceivedLine) // Line is too short, following lines will be too short, too.
      return false;
    Vector2f dir = line.last - line.first;
    dir.normalize();
    const Geometry::Line geoLine(line.first, dir);
    if(Geometry::getDistanceToLine(geoLine, centerCirclePosition) < maxLineDeviationFromCenter)
    {
      centerLine = geoLine;
      return true;
    }
  }
  return false;
}

void CenterCircleWithLinePerceptor::computePose(CenterCircleWithLine& centerCircleWithLine)
{
  Vector2f lineDirection = centerLine.direction;
  lineDirection.rotateLeft();
  Pose2f perceivedPose(lineDirection.angle(), centerCirclePosition);
  centerCircleWithLine = perceivedPose;
}

MAKE_MODULE(CenterCircleWithLinePerceptor, perception);
