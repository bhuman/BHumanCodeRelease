/**
 * @file MidCirclePerceptor.cpp
 * Provides MidCircle.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "MidCirclePerceptor.h"
#include "Tools/Math/Transformation.h"
#include "Math/Geometry.h"
#include "Math/BHMath.h"
#include <algorithm>

void MidCirclePerceptor::update(MidCircle& midCircle)
{
  if(theGameState.isPenaltyShootout())
  {
    midCircle.isValid = false;
    return;
  }
  midCircle.isValid = searchCircleWithLine(midCircle);
  lastFrameTime = theFrameInfo.time;
  theLastCirclePercept = theCirclePercept;
}

bool MidCirclePerceptor::searchCircleWithLine(MidCircle& midCircle) const
{
  if(!(theCirclePercept.wasSeen
       || (theLastCirclePercept.wasSeen
           && theFrameInfo.getTimeSince(lastFrameTime) <= maxTimeOffset // because of log forwards jumps (and missing images)
           && theFrameInfo.time >= lastFrameTime) // because of logs backwards jumps
      ))
    return false;

  const Vector2f theMidCirclePosition = theCirclePercept.wasSeen ? theCirclePercept.pos : theOdometer.odometryOffset.inverse() * theLastCirclePercept.pos;

  // sorting lines (or rather line access) from long to short
  std::vector<int> sortedLinePointers;
  for(unsigned i = 0; i < theFieldLines.lines.size(); i++)
    sortedLinePointers.emplace_back(i);
  std::sort(sortedLinePointers.begin(), sortedLinePointers.end(), [&](const int a, const int b)
  {
    return
      (theFieldLines.lines[a].first - theFieldLines.lines[a].last).squaredNorm() > (theFieldLines.lines[b].first - theFieldLines.lines[b].last).squaredNorm();
  });

  for(int i : sortedLinePointers)
  {
    if(squaredMinLineLength > (theFieldLines.lines[i].last - theFieldLines.lines[i].first).squaredNorm())
      continue;

    const Geometry::Line geomLine(theFieldLines.lines[i].first, theFieldLines.lines[i].last - theFieldLines.lines[i].first);
    const float distanceToLine = Geometry::getDistanceToLine(geomLine, theMidCirclePosition);

    const float lineSquaredNorm = (theFieldLines.lines[i].first - theFieldLines.lines[i].last).squaredNorm();
    const float squaredFirstDist = (theMidCirclePosition - theFieldLines.lines[i].first).squaredNorm();
    const float squaredLastDist = (theMidCirclePosition - theFieldLines.lines[i].last).squaredNorm();

    auto rejectVerticallyLine = [&]
    {
      if(std::abs(std::abs(geomLine.direction.rotated(-theJointAngles.angles[Joints::headYaw]).angle()) - 90_deg) < 60_deg) // vertical line is defined here as +- 30deg
        return false;

      Vector2f leftBottomCameraEdgeOnTheField, rightBottomCameraEdgeOnTheField;

      if(!theCirclePercept.wasSeen
         || !Transformation::imageToRobot(0, theCameraInfo.height, theCameraMatrix, theCameraInfo, leftBottomCameraEdgeOnTheField)
         || !Transformation::imageToRobot(theCameraInfo.width, theCameraInfo.height, theCameraMatrix, theCameraInfo, rightBottomCameraEdgeOnTheField))
        return true;

      const float distanceCameraLineToCircle = Geometry::getDistanceToLineSigned(
        Geometry::Line(leftBottomCameraEdgeOnTheField, rightBottomCameraEdgeOnTheField - leftBottomCameraEdgeOnTheField), theMidCirclePosition);

      Vector2f unused;
      if(distanceCameraLineToCircle > -theFieldDimensions.centerCircleRadius / 2
         && std::min(squaredFirstDist, squaredLastDist) < sqr(theFieldDimensions.centerCircleRadius / 2))
        return false;

      return true;
    };

    if(distanceToLine < maxLineDistanceToCircleCenter &&
       (squaredFirstDist < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + allowedOffsetOfMidLineEndToCircleCenter)) ||
        squaredLastDist  < sqr(std::max(0.f, theFieldDimensions.centerCircleRadius + allowedOffsetOfMidLineEndToCircleCenter)) ||
        (squaredFirstDist <= lineSquaredNorm && squaredLastDist <= lineSquaredNorm))
       && !rejectVerticallyLine())
    {
      midCircle.translation = theMidCirclePosition;
      midCircle.rotation = theFieldLines.lines[i].alpha;
      return true;
    }
  }
  return false;
}

MAKE_MODULE(MidCirclePerceptor, perception);
