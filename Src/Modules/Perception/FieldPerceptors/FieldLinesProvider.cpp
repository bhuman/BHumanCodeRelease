/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldLinesProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Geometry.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"

using namespace std;

MAKE_MODULE(FieldLinesProvider, perception)

bool FieldLinesProvider::isPointInSegment(const SpotLine& line, const Vector2f& point) const
{
  const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
  return (line.firstField - point).squaredNorm() <= lineSquaredNorm && (line.lastField - point).squaredNorm() <= lineSquaredNorm;
}

void FieldLinesProvider::update(FieldLines& fieldLines)
{
  fieldLines.lines.clear();
  spotLineUsage.clear();
  lineIndexTable.clear();

  midLine = 0;

  for(const SpotLine& line : theLinesPercept.lines)
  {
    if(line.belongsToCircle)
    {
      //FieldLines should not contain lines that are on the circle
      spotLineUsage.push_back(thrown);
      lineIndexTable.push_back(lostIndex);
      continue;
    }

    {
      //is line most likely in/of center circle
      if((theCirclePercept.wasSeen || lastCircleWasSeen)
         && theFrameInfo.getTimeSince(lastFrameTime) <= maxTimeOffset) // because of log backjumps
      {
        const Vector2f theMidCirclePosition = theCirclePercept.wasSeen ? theCirclePercept.pos : theOdometer.odometryOffset * theCirclePercept.pos;

        if(std::abs(std::abs(Geometry::getDistanceToLine(line.line, theMidCirclePosition)) - theFieldDimensions.centerCircleRadius) < lineOfCircleAssumationVariance)
        {
          //(again) FieldLines should not contain lines that are on the circle
          spotLineUsage.push_back(thrown);
          lineIndexTable.push_back(lostIndex);
          continue;
        }
      }
    }

    {
      //remove false-positives in balls
      const Vector2i centerInImage = (line.firstImg + line.lastImg) / 2;
      const float inImageRadius = IISC::getImageBallRadiusByCenter(centerInImage.cast<float>(), theCameraInfo, theCameraMatrix, theBallSpecification);
      if((line.firstImg - line.lastImg).squaredNorm() < static_cast<int>(sqr(inImageRadius * 2.5f) * std::abs(std::cos(line.line.direction.angle()))))
      {
        spotLineUsage.push_back(thrown);
        lineIndexTable.push_back(lostIndex);
        continue;
      }
    }

    Vector2f firstInField(line.firstField);
    Vector2f lastInField(line.lastField);
    spotLineUsage.emplace_back(stayed);

    if(theGoalFrame.isGroundLineValid)
    {
      const Vector2f firstInGF(theGoalFrame.inverse() * line.firstField);
      const Vector2f lastInGF(theGoalFrame.inverse() * line.lastField);

      Vector2f intersection;

      if(firstInGF.x() > goalFrameThreshold)
      {
        if(Geometry::getIntersectionOfLines(
             Geometry::Line(Vector2f(lastInGF.x() > 0.f ? goalFrameThreshold : 0.f, 0.f), Vector2f(0.f, 1.f)),
             Geometry::Line(firstInGF, firstInGF - lastInGF), intersection))
        {
          firstInField = theGoalFrame * intersection;
          spotLineUsage.back() = cutted;
        }
        else
        {
          spotLineUsage.back() = thrown;
          lineIndexTable.push_back(lostIndex);
          continue;
        }
      }

      if(lastInGF.x() > goalFrameThreshold)
      {
        if(Geometry::getIntersectionOfLines(
             Geometry::Line(Vector2f(firstInGF.x() > 0.f ? goalFrameThreshold : 0.f, 0.f), Vector2f(0.f, 1.f)),
             Geometry::Line(lastInGF, lastInGF - firstInGF), intersection))
        {
          lastInField = theGoalFrame * intersection;
          spotLineUsage.back() = cutted;
        }
        else
        {
          spotLineUsage.back() = thrown;
          lineIndexTable.push_back(lostIndex);
          continue;
        }
      }

      if((firstInField - lastInField).squaredNorm() < squaredMinLenghOfACuttedLine)
      {
        spotLineUsage.back() = thrown;
        lineIndexTable.push_back(lostIndex);
        continue;
      }
    }

    ASSERT(std::isfinite(firstInField.x()));
    ASSERT(std::isfinite(firstInField.y()));
    ASSERT(std::isfinite(lastInField.x()));
    ASSERT(std::isfinite(lastInField.y()));

    fieldLines.lines.emplace_back();
    PerceptLine& pLine = fieldLines.lines.back();
    lineIndexTable.push_back(static_cast<unsigned>(fieldLines.lines.size() - 1));
    pLine.midLine = false;
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
            (firstInField - lastInField).squaredNorm() > squaredBigLineThreshold / 2))
        {
          pLine.midLine = true;
          midLine = const_cast<SpotLine*>(&line);
        }
      }
    }

    pLine.first = firstInField;
    pLine.last = lastInField;

    pLine.isLong = spotLineUsage.back() != cutted && (firstInField - lastInField).squaredNorm() > squaredBigLineThreshold;

    pLine.alpha = Vector2f(pLine.last - pLine.first).rotateLeft().angle();
    pLine.d = Geometry::getDistanceToLine(Geometry::Line(pLine.first, (pLine.last - pLine.first)), Vector2f::Zero());
  }
  lastCircleWasSeen = theCirclePercept.wasSeen;
  lastFrameTime = theFrameInfo.time;
}

void FieldLinesProvider::update(FieldLineIntersections& fieldLineIntersections)
{
  fieldLineIntersections.intersections.clear();

  if(SystemCall::getMode() == SystemCall::logFileReplay && spotLineUsage.size() != theLinesPercept.lines.size())
    return;
  ASSERT(spotLineUsage.size() == theLinesPercept.lines.size());
  ASSERT(lineIndexTable.size() == theLinesPercept.lines.size());

  for(const IntersectionsPercept::Intersection& i : theIntersectionsPercept.intersections)
  {
    if(theGoalFrame.isGroundLineValid)
    {
      const Vector2f intersectionInGF(theGoalFrame.inverse() * i.pos);
      if(intersectionInGF.x() > goalFrameThreshold)
        continue;
    }

    if(spotLineUsage[i.line1Index] == thrown || spotLineUsage[i.line2Index] == thrown)
      continue;

    fieldLineIntersections.intersections.emplace_back();
    fieldLineIntersections.intersections.back().ownIndex = unsigned(fieldLineIntersections.intersections.size()) - 1u;
    fieldLineIntersections.intersections.back().pos = i.pos;
    if(spotLineUsage[i.line1Index] == cutted || spotLineUsage[i.line2Index] == cutted)
      switch(i.type)
      {
        case IntersectionsPercept::Intersection::L:
          fieldLineIntersections.intersections.pop_back();
          continue;
        case IntersectionsPercept::Intersection::T:
          if(spotLineUsage[i.line1Index] == cutted)
          {
            fieldLineIntersections.intersections.pop_back();
            continue;
          }
          else
          {
            fieldLineIntersections.intersections.back().type = FieldLineIntersections::Intersection::L;
            //TODO FIXME
            break;
          }
        case IntersectionsPercept::Intersection::X:
          //FIXME
          fieldLineIntersections.intersections.back().type = FieldLineIntersections::Intersection::T;
          break;
        default:
          break;
      }
    else
      fieldLineIntersections.intersections.back().type = static_cast<FieldLineIntersections::Intersection::IntersectionType>(i.type);
    fieldLineIntersections.intersections.back().dir1 = i.dir1.normalized();
    fieldLineIntersections.intersections.back().indexDir1 = lineIndexTable[i.line1Index];
    fieldLineIntersections.intersections.back().dir2 = i.dir2.normalized();
    fieldLineIntersections.intersections.back().indexDir2 = lineIndexTable[i.line2Index];
    ASSERT(fieldLineIntersections.intersections.back().indexDir1 != fieldLineIntersections.intersections.back().indexDir2
           && fieldLineIntersections.intersections.back().indexDir1 != lostIndex
           && fieldLineIntersections.intersections.back().indexDir2 != lostIndex);

    auto equalLines = [](const SpotLine* l1, const SpotLine* l2)
    {
      return l1->firstField == l2->firstField && l1->lastField == l2->lastField;
    };

    const bool line1IsMid = midLine && equalLines(midLine, &theLinesPercept.lines[i.line1Index]);
    const bool line2IsMid = midLine && equalLines(midLine, &theLinesPercept.lines[i.line2Index]);

    ASSERT(!line1IsMid || !line2IsMid);
    if((i.type == IntersectionsPercept::Intersection::T && line2IsMid) || theLinesPercept.lines[i.line1Index].belongsToCircle || theLinesPercept.lines[i.line2Index].belongsToCircle)
      continue;

    if(line1IsMid || line2IsMid)
    {
      fieldLineIntersections.intersections.back().additionalType = FieldLineIntersections::Intersection::mid;
      /*
      if(theLinesPercept.lines[i.line1Index].belongsToCircle || theLinesPercept.lines[i.line2Index].belongsToCircle) //maybe delete this
      {
        fieldLineIntersections.intersections.back().type = FieldLineIntersections::Intersection::X;
      }
      else if(i.type == IntersectionsPercept::Intersection::L)
      {
        fieldLineIntersections.intersections.back().type = FieldLineIntersections::Intersection::T;
        if(line1IsMid) //NOTE: see note before
        {
        const Vector2f temp = fieldLines.intersections.back().dir1;
        fieldLines.intersections.back().dir1 = fieldLines.intersections.back().dir2;
        fieldLines.intersections.back().dir2 = temp;
        }
      }*/
    }
    else if(theFieldLines.lines[fieldLineIntersections.intersections.back().indexDir1].isLong
            && theFieldLines.lines[fieldLineIntersections.intersections.back().indexDir2].isLong)
      fieldLineIntersections.intersections.back().additionalType = FieldLineIntersections::Intersection::big;

    if(fieldLineIntersections.intersections.back().type == FieldLineIntersections::Intersection::X &&
       fieldLineIntersections.intersections.back().additionalType == FieldLineIntersections::Intersection::big &&
       (theLinesPercept.lines[i.line1Index].firstField - theLinesPercept.lines[i.line1Index].lastField).squaredNorm() < (theLinesPercept.lines[i.line2Index].firstField - theLinesPercept.lines[i.line2Index].lastField).squaredNorm()) //this will be "asserted" in GoalFramePerceptor
    {
      const Vector2f temp = fieldLineIntersections.intersections.back().dir1;
      fieldLineIntersections.intersections.back().dir1 = fieldLineIntersections.intersections.back().dir2;
      fieldLineIntersections.intersections.back().dir2 = temp;

      fieldLineIntersections.intersections.back().indexDir2 += fieldLineIntersections.intersections.back().indexDir1;
      fieldLineIntersections.intersections.back().indexDir1 = fieldLineIntersections.intersections.back().indexDir2 - fieldLineIntersections.intersections.back().indexDir1;
      fieldLineIntersections.intersections.back().indexDir2 -= fieldLineIntersections.intersections.back().indexDir1;
    }

    //ensure dir2 is left of dir1
    if(fieldLineIntersections.intersections.back().type == FieldLineIntersections::Intersection::L &&
       Angle(fieldLineIntersections.intersections.back().dir2.angle() - 90_deg).normalize().diffAbs(fieldLineIntersections.intersections.back().dir1.angle()) > 90_deg)
    {
      const Vector2f temp = fieldLineIntersections.intersections.back().dir2;
      fieldLineIntersections.intersections.back().dir2 = fieldLineIntersections.intersections.back().dir1;
      fieldLineIntersections.intersections.back().dir1 = temp;

      fieldLineIntersections.intersections.back().indexDir2 += fieldLineIntersections.intersections.back().indexDir1;
      fieldLineIntersections.intersections.back().indexDir1 = fieldLineIntersections.intersections.back().indexDir2 - fieldLineIntersections.intersections.back().indexDir1;
      fieldLineIntersections.intersections.back().indexDir2 -= fieldLineIntersections.intersections.back().indexDir1;
    }

    ASSERT(fieldLineIntersections.intersections.back().indexDir1 != fieldLineIntersections.intersections.back().indexDir2
           && fieldLineIntersections.intersections.back().indexDir1 != lostIndex
           && fieldLineIntersections.intersections.back().indexDir2 != lostIndex);
  }
}
