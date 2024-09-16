/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldLinesProvider.h"
#include "Platform/SystemCall.h"
#include "Math/Geometry.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

#include <algorithm>

MAKE_MODULE(FieldLinesProvider);

bool FieldLinesProvider::isPointInSegment(const SpotLine& line, const Vector2f& point) const
{
  const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
  return (line.firstField - point).squaredNorm() <= lineSquaredNorm && (line.lastField - point).squaredNorm() <= lineSquaredNorm;
}

void FieldLinesProvider::update(FieldLines& fieldLines)
{
  internalListOfLines.clear();
  spotLineUsage.clear();

  midLine = nullptr;

  for(const SpotLine& line : theLinesPercept.lines)
  {
    if(theGameState.isPenaltyShootout())
    {
      if(std::abs(std::abs((theRobotPose * line.firstField - theRobotPose * line.lastField).angle()) - 90_deg) > 30_deg
         && (std::abs((theRobotPose * line.firstField).y()) < theFieldDimensions.yPosLeftPenaltyArea - 150
             || std::abs((theRobotPose * line.lastField).y()) < theFieldDimensions.yPosLeftPenaltyArea - 150))
      {
        spotLineUsage.push_back(thrown);
        continue;
      }

      const Vector2f corner1(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder - 200);
      const Vector2f corner2(theFieldDimensions.xPosPenaltyStrikerStartPosition / 2, theFieldDimensions.yPosRightFieldBorder - 200);
      if(!Geometry::isPointInsideRectangle2(corner1, corner2, theRobotPose * line.firstField)
         || !Geometry::isPointInsideRectangle2(corner1, corner2, theRobotPose * line.lastField))
      {
        spotLineUsage.push_back(thrown);
        continue;
      }
    }

    if(line.belongsToCircle)
    {
      //FieldLines should not contain lines that are on the circle
      spotLineUsage.push_back(thrown);
      continue;
    }

    {
      //is line most likely in/of center circle
      if((theCirclePercept.wasSeen || lastCirclePercept.wasSeen)
         && theFrameInfo.getTimeSince(lastFrameTime) <= maxTimeOffset) // because of log backjumps
      {
        const Vector2f centerCirclePosition = theCirclePercept.wasSeen ? theCirclePercept.pos : theOdometryData.inverse() * lastOdometryData * lastCirclePercept.pos;

        if(std::abs(Geometry::getDistanceToLine(line.line, centerCirclePosition) - theFieldDimensions.centerCircleRadius) < maxLineDeviationFromAssumedCenterCircle)
        {
          //(again) FieldLines should not contain lines that are on the circle
          spotLineUsage.push_back(thrown);
          continue;
        }
      }
    }

    {
      //remove false-positives in balls
      const Vector2i centerInImage = (line.firstImg + line.lastImg) / 2;
      Vector2f relativePosition;
      Geometry::Circle ball;
      const float inImageRadius = Transformation::imageToRobotHorizontalPlane(centerInImage.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
                                  && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
      if((line.firstImg - line.lastImg).squaredNorm() < static_cast<int>(sqr(inImageRadius * 2.5f) * std::abs(std::cos(line.line.direction.angle()))))
      {
        spotLineUsage.push_back(thrown);
        continue;
      }
    }

    Vector2f firstInField(line.firstField);
    Vector2f lastInField(line.lastField);
    spotLineUsage.emplace_back(stayed);

    ASSERT(std::isfinite(firstInField.x()));
    ASSERT(std::isfinite(firstInField.y()));
    ASSERT(std::isfinite(lastInField.x()));
    ASSERT(std::isfinite(lastInField.y()));

    internalListOfLines.emplace_back();
    PerceptLine& pLine = internalListOfLines.back();
    if(theCirclePercept.wasSeen)
    {
      //check if this is the midline
      Vector2f intersectionA;
      Vector2f intersectionB;
      const int numIntersections = Geometry::getIntersectionOfLineAndCircle(line.line,
                                   Geometry::Circle(theCirclePercept.pos, theFieldDimensions.centerCircleRadius), intersectionA, intersectionB);
      if(numIntersections == 2)
      {
        //if the line intersects the circle twice
        //check if both intersections are between lineStart and lineEnd
        //       Note(jesse): changes to one intersection is enough
        if(isPointInSegment(line, intersectionA) || isPointInSegment(line, intersectionB) ||
           ((theCirclePercept.pos - line.firstField).squaredNorm() < sqr(theFieldDimensions.centerCircleRadius) &&
            (theCirclePercept.pos - line.lastField).squaredNorm() < sqr(theFieldDimensions.centerCircleRadius) &&
            (firstInField - lastInField).squaredNorm() > bigLineThreshold*bigLineThreshold / 2))
        {
          midLine = &line;
        }
      }
    }
    // Fill elements of FieldLine representation:
    pLine.first = firstInField;
    pLine.last = lastInField;
    pLine.length = (firstInField - lastInField).norm();
    pLine.alpha = Vector2f(pLine.last - pLine.first).rotateLeft().angle();
    const Vector2f linePerceptCenter = (pLine.first + pLine.last) * 0.5f;
    const Vector2f linePerceptClosestPoint = Geometry::getOrthogonalProjectionOfPointOnEdge(pLine.first, pLine.last - pLine.first, Vector2f(0.f,0.f));
    float closestWeighting = 1.f - centerWeighting;
    pLine.cov = theMeasurementCovariance.computeForRelativePosition(linePerceptCenter*centerWeighting + linePerceptClosestPoint*closestWeighting);
  }
  // Sort final list of lines from long to short:
  std::vector<size_t> sortedLineIndizes;
  for(size_t i = 0; i < internalListOfLines.size(); i++)
    sortedLineIndizes.emplace_back(i);
  std::sort(sortedLineIndizes.begin(), sortedLineIndizes.end(), [&](const size_t a, const size_t b)
  {
    return internalListOfLines[a].length > internalListOfLines[b].length;
  });
  // Copy sorted list to representation:
  fieldLines.lines.clear();
  for(size_t i = 0; i < sortedLineIndizes.size(); i++)
    fieldLines.lines.emplace_back(internalListOfLines[sortedLineIndizes[i]]);
  lastCirclePercept = theCirclePercept;
  lastOdometryData = theOdometryData;
  lastFrameTime = theFrameInfo.time;
}

void FieldLinesProvider::update(FieldLineIntersections& fieldLineIntersections)
{
  fieldLineIntersections.intersections.clear();

  if(SystemCall::getMode() == SystemCall::logFileReplay && spotLineUsage.size() != theLinesPercept.lines.size())
    return;
  ASSERT(spotLineUsage.size() == theLinesPercept.lines.size());

  for(const IntersectionsPercept::Intersection& i : theIntersectionsPercept.intersections)
  {
    if(spotLineUsage[i.line1Index] == thrown || spotLineUsage[i.line2Index] == thrown)
      continue;

    fieldLineIntersections.intersections.emplace_back();
    fieldLineIntersections.intersections.back().pos = i.pos;
    fieldLineIntersections.intersections.back().cov = i.cov;
    fieldLineIntersections.intersections.back().type = static_cast<FieldLineIntersections::Intersection::IntersectionType>(i.type);
    fieldLineIntersections.intersections.back().dir1 = i.dir1.normalized();
    fieldLineIntersections.intersections.back().dir2 = i.dir2.normalized();

    auto equalLines = [](const SpotLine* l1, const SpotLine* l2)
    {
      return l1->firstField == l2->firstField && l1->lastField == l2->lastField;
    };

    [[maybe_unused]] const bool line1IsMid = midLine && equalLines(midLine, &theLinesPercept.lines[i.line1Index]);
    const bool line2IsMid = midLine && equalLines(midLine, &theLinesPercept.lines[i.line2Index]);

    ASSERT(!line1IsMid || !line2IsMid);
    if((i.type == IntersectionsPercept::Intersection::T && line2IsMid) || theLinesPercept.lines[i.line1Index].belongsToCircle || theLinesPercept.lines[i.line2Index].belongsToCircle)
      continue;

    //ensure dir2 is left of dir1
    if(fieldLineIntersections.intersections.back().type == FieldLineIntersections::Intersection::L &&
       Angle(fieldLineIntersections.intersections.back().dir2.angle() - 90_deg).normalize().diffAbs(fieldLineIntersections.intersections.back().dir1.angle()) > 90_deg)
    {
      const Vector2f temp = fieldLineIntersections.intersections.back().dir2;
      fieldLineIntersections.intersections.back().dir2 = fieldLineIntersections.intersections.back().dir1;
      fieldLineIntersections.intersections.back().dir1 = temp;
    }
  }
}
