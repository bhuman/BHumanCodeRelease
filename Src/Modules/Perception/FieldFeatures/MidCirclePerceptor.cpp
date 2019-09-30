/**
 * @file MidCirclePerceptor.cpp
 * Provides MidCircle.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "MidCirclePerceptor.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/BHMath.h"
#include <algorithm>

void MidCirclePerceptor::update(MidCircle& midCircle)
{
  midCircle.clear();

  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    midCircle.isValid = false;
    return;
  }

  if(searchCircleWithLine(midCircle) ||
     searchWithSXAndT(midCircle))
    midCircle.isValid = true;
  else
    midCircle.isValid = false;

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

  const Vector2f theMidCirclePosition = theCirclePercept.wasSeen ? theCirclePercept.pos : theOdometer.odometryOffset * theLastCirclePercept.pos;

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
    const float rawDistanceToLine = Geometry::getDistanceToLine(geomLine, theMidCirclePosition);
    const float distanceToLine = std::abs(rawDistanceToLine);

    const float lineSquaredNorm = (theFieldLines.lines[i].first - theFieldLines.lines[i].last).squaredNorm();
    const float squaredFirstDist = (theMidCirclePosition - theFieldLines.lines[i].first).squaredNorm();
    const float squaredLastDist = (theMidCirclePosition - theFieldLines.lines[i].last).squaredNorm();

    auto rejectVerticallyLine = [&]()
    {
      if(std::abs(std::abs(geomLine.direction.rotated(-theJointAngles.angles[Joints::headYaw]).angle()) - 90_deg) < 60_deg) // vertical line is defined here as +- 30deg
        return false;

      Vector2f leftBottomCameraEdgeOnTheField, rightBottomCameraEdgeOnTheField;

      if(!theCirclePercept.wasSeen
         || !Transformation::imageToRobot(0, theCameraInfo.height, theCameraMatrix, theCameraInfo, leftBottomCameraEdgeOnTheField)
         || !Transformation::imageToRobot(theCameraInfo.width, theCameraInfo.height, theCameraMatrix, theCameraInfo, rightBottomCameraEdgeOnTheField))
        return true;

      const float distanceCameraLineToCircle = Geometry::getDistanceToLine(
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

      midCircle.markedPoints.emplace_back(theMidCirclePosition, MarkedPoint::midCircle, theCirclePercept.wasSeen);
      theIntersectionRelations.propagateMarkedLinePoint(MarkedLine(i, MarkedLine::midLine), 0.f, theMidCirclePosition,
          theFieldLineIntersections, theFieldLines, midCircle);

      return true;
    }
  }
  return false;
}

bool MidCirclePerceptor::searchWithSXAndT(MidCircle& midCircle) const
{
  std::vector<const FieldLineIntersections::Intersection*> useSmallXIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::X && intersection.additionalType == FieldLineIntersections::Intersection::none)
      useSmallXIntersections.push_back(&intersection);

  if(useSmallXIntersections.empty())
    return false;

  static const float distance = theFieldDimensions.yPosLeftSideline - theFieldDimensions.centerCircleRadius   ;

  std::vector<const FieldLineIntersections::Intersection*> useTIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::T)
      useTIntersections.push_back(&intersection);

  for(const FieldLineIntersections::Intersection* intersectionSX : useSmallXIntersections)
    for(const FieldLineIntersections::Intersection* intersectionT : useTIntersections)
      if(intersectionT->indexDir1 == intersectionSX->indexDir1 || intersectionT->indexDir1 == intersectionSX->indexDir2)
        if(std::abs((intersectionT->pos - intersectionSX->pos).norm() - distance) < allowedTsXVariance)
        {
          const Vector2f dirTToX = intersectionSX->pos - intersectionT->pos;
          midCircle.translation = intersectionSX->pos + dirTToX.normalized(theFieldDimensions.centerCircleRadius);
          midCircle.rotation = Angle(dirTToX.angle() + pi_2).normalize();// OR  intersectionT->dir2.angle();

          midCircle.markedPoints.emplace_back(midCircle.translation, MarkedPoint::midCircle);
          theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersectionT->ownIndex, MarkedIntersection::BT),
              theFieldLineIntersections, theFieldLines, midCircle);

          return true;
        }

  return false;
}

MAKE_MODULE(MidCirclePerceptor, perception)
