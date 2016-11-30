#include "MidCirclePerceptor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/BHMath.h"

void MidCirclePerceptor::update(MidCircle& midCircle)
{
  midCircle.clear();

  if(searchCircleWithLine(midCircle) ||
     searchWithSXAndT(midCircle))
    midCircle.isValid = true;
  else
    midCircle.isValid = false;

  lastFrameTime = theFrameInfo.time;
}

bool MidCirclePerceptor::searchCircleWithLine(MidCircle& midCircle) const
{
  if(theFrameInfo.getTimeSince(theCirclePercept.lastSeen) > theFrameInfo.getTimeSince(lastFrameTime)
    || theFrameInfo.getTimeSince(theCirclePercept.lastSeen) > maxTimeOffset
    || theFrameInfo.time < theCirclePercept.lastSeen) //because of logs backjumps
    return false;

  const Vector2f theMidCirclePosition = theFrameInfo.getTimeSince(theCirclePercept.lastSeen) > 0 ? theOdometer.odometryOffset * theCirclePercept.pos : theCirclePercept.pos;

  for(unsigned i = 0; i < theFieldLines.lines.size(); i++)
  {
    if(squaredMinLineLength > (theFieldLines.lines[i].last - theFieldLines.lines[i].first).squaredNorm())
      continue;

    const Geometry::Line geomLine(theFieldLines.lines[i].first, theFieldLines.lines[i].last - theFieldLines.lines[i].first);
    const float rawDistanceToLine = Geometry::getDistanceToLine(geomLine, theMidCirclePosition);
    const float distanceToLine = std::abs(rawDistanceToLine);

    const float lineSquaredNorm = (theFieldLines.lines[i].first - theFieldLines.lines[i].last).squaredNorm();
    const float squaredFirstDist = (theMidCirclePosition - theFieldLines.lines[i].first).squaredNorm();
    const float squaredLastDist = (theMidCirclePosition - theFieldLines.lines[i].last).squaredNorm();

    if(distanceToLine < maxLineDistanceToCircleCenter &&
       (squaredFirstDist < sqr(theFieldDimensions.centerCircleRadius + allowedOffsetOfMidLineEndToCircleCenter) ||
        squaredLastDist < sqr(theFieldDimensions.centerCircleRadius + allowedOffsetOfMidLineEndToCircleCenter) ||
        (squaredFirstDist <= lineSquaredNorm && squaredLastDist <= lineSquaredNorm)))
    {
      midCircle.translation = theMidCirclePosition;
      midCircle.rotation = theFieldLines.lines[i].alpha;

      midCircle.markedPoints.emplace_back(theMidCirclePosition, MarkedPoint::midCircle, theFrameInfo.getTimeSince(theCirclePercept.lastSeen) == 0);
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
