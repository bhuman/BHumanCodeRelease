/**
 * @file FieldRelations.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldRelations.h"
#include "Platform/BHAssert.h"
#include <vector>

IntersectionRelations::IntersectionLineRelations::IntersectionLineRelations()
{
  lineMarkerOfDir1[MarkedIntersection::BT] = MarkedLine::midLine;
  lineMarkerOfDir2[MarkedIntersection::BT] = MarkedLine::sideLine;
  lineMarkerOfDir1[MarkedIntersection::STL] = MarkedLine::sidePenaltyL;
  lineMarkerOfDir2[MarkedIntersection::STL] = MarkedLine::groundLine;
  lineMarkerOfDir1[MarkedIntersection::STR] = MarkedLine::sidePenaltyR;
  lineMarkerOfDir2[MarkedIntersection::STR] = MarkedLine::groundLine;
  lineMarkerOfDir1[MarkedIntersection::BLL] = MarkedLine::sideLine;
  lineMarkerOfDir2[MarkedIntersection::BLL] = MarkedLine::groundLine;
  lineMarkerOfDir1[MarkedIntersection::SLL] = MarkedLine::groundPenalty;
  lineMarkerOfDir2[MarkedIntersection::SLL] = MarkedLine::sidePenaltyL;
  lineMarkerOfDir1[MarkedIntersection::SLR] = MarkedLine::sidePenaltyR;
  lineMarkerOfDir2[MarkedIntersection::SLR] = MarkedLine::groundPenalty;
  lineMarkerOfDir1[MarkedIntersection::BLR] = MarkedLine::groundLine;
  lineMarkerOfDir2[MarkedIntersection::BLR] = MarkedLine::otherSideLine;
};

IntersectionRelations::LineIntersectionRelations::LineIntersectionRelations(const FieldDimensions& theFieldDimensions)
{
  intersections[MarkedLine::midLine][0] = MarkedIntersection::BT;
  intersections[MarkedLine::midLine][1] = MarkedIntersection::otherBT;
  intersections[MarkedLine::midLine][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::midLine][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::groundLine][0] = MarkedIntersection::BLL;
  intersections[MarkedLine::groundLine][1] = MarkedIntersection::STL;
  intersections[MarkedLine::groundLine][2] = MarkedIntersection::STR;
  intersections[MarkedLine::groundLine][3] = MarkedIntersection::BLR;
  intersections[MarkedLine::sideLine][0] = MarkedIntersection::BLL;
  intersections[MarkedLine::sideLine][1] = MarkedIntersection::BT;
  intersections[MarkedLine::sideLine][2] = MarkedIntersection::otherBLR;
  intersections[MarkedLine::sideLine][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::groundPenalty][0] = MarkedIntersection::SLL;
  intersections[MarkedLine::groundPenalty][1] = MarkedIntersection::SLR;
  intersections[MarkedLine::groundPenalty][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::groundPenalty][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::sidePenaltyL][0] = MarkedIntersection::STL;
  intersections[MarkedLine::sidePenaltyL][1] = MarkedIntersection::SLL;
  intersections[MarkedLine::sidePenaltyL][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::sidePenaltyL][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::sidePenaltyR][0] = MarkedIntersection::SLR;
  intersections[MarkedLine::sidePenaltyR][1] = MarkedIntersection::STR;
  intersections[MarkedLine::sidePenaltyR][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersections[MarkedLine::sidePenaltyR][3] = MarkedIntersection::numOfIntersectionMarkers;

  intersectionPositions[MarkedLine::midLine][0] = theFieldDimensions.yPosLeftSideline;
  intersectionPositions[MarkedLine::midLine][1] = -intersectionPositions[MarkedLine::midLine][0];
  intersectionPositions[MarkedLine::midLine][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::midLine][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::groundLine][0] = theFieldDimensions.yPosLeftSideline;
  intersectionPositions[MarkedLine::groundLine][1] = theFieldDimensions.yPosLeftPenaltyArea;
  intersectionPositions[MarkedLine::groundLine][2] = -intersectionPositions[MarkedLine::groundLine][1];
  intersectionPositions[MarkedLine::groundLine][3] = -intersectionPositions[MarkedLine::groundLine][0];
  intersectionPositions[MarkedLine::sideLine][0] = theFieldDimensions.xPosOpponentGroundline;
  intersectionPositions[MarkedLine::sideLine][1] = theFieldDimensions.xPosHalfWayLine;
  intersectionPositions[MarkedLine::sideLine][2] = -intersectionPositions[MarkedLine::sideLine][0];
  intersectionPositions[MarkedLine::sideLine][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::groundPenalty][0] = theFieldDimensions.yPosLeftPenaltyArea;
  intersectionPositions[MarkedLine::groundPenalty][1] = -intersectionPositions[MarkedLine::groundPenalty][0];
  intersectionPositions[MarkedLine::groundPenalty][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::groundPenalty][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::sidePenaltyL][0] = theFieldDimensions.xPosOpponentGroundline;
  intersectionPositions[MarkedLine::sidePenaltyL][1] = theFieldDimensions.xPosOpponentPenaltyArea;
  intersectionPositions[MarkedLine::sidePenaltyL][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::sidePenaltyL][3] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::sidePenaltyR][0] = theFieldDimensions.xPosOpponentPenaltyArea;
  intersectionPositions[MarkedLine::sidePenaltyR][1] = theFieldDimensions.xPosOpponentGroundline;
  intersectionPositions[MarkedLine::sidePenaltyR][2] = MarkedIntersection::numOfIntersectionMarkers;
  intersectionPositions[MarkedLine::sidePenaltyR][3] = MarkedIntersection::numOfIntersectionMarkers;
}

void IntersectionRelations::LineIntersectionRelations::serialize(In* in, Out* out)
{
  for(int i = 0; i < MarkedLine::firstLineMarkerOther; ++i)
    Streaming::streamIt(in, out, (std::string("intersections") + std::to_string(i)).data(), intersections[i]);
  for(int i = 0; i < MarkedLine::firstLineMarkerOther; ++i)
    Streaming::streamIt(in, out, (std::string("intersectionPositions") + std::to_string(i)).data(), intersectionPositions[i]);
}

void IntersectionRelations::propagateMarkedIntersection(const MarkedIntersection& start, const FieldLineIntersections& theFieldLineIntersections,
    const FieldLines& theFieldLines, FieldFeature& ff) const
{
  std::vector<bool> usedIntersection(theFieldLineIntersections.intersections.size(), false);
  usedIntersection[start.intersectionIndex] = true;
  std::vector<bool> usedLines(theFieldLines.lines.size(), false);

  std::vector<MarkedIntersection> handleIntersections;
  handleIntersections.emplace_back(start);

  while(!handleIntersections.empty())
  {
    ff.markedIntersections.emplace_back(handleIntersections.back());

    const MarkedIntersection& currentMI = ff.markedIntersections.back();
    const FieldLineIntersections::Intersection& currIntersection = theFieldLineIntersections.intersections[currentMI.intersectionIndex];
    handleIntersections.pop_back();

    auto findIntersections = [&](const MarkedLine& mLine)
    {
      usedLines[mLine.lineIndex] = true;
      const bool mirror = mLine.marker >= MarkedLine::firstLineMarkerOther;
      const MarkedLine::LineMarker calcMLineMarker = mirror ? mLine.mirror() : mLine.marker;
      const MarkedIntersection::IntersectionMarker calcCurrMarker = mirror ? currentMI.mirror() : currentMI.marker;

      unsigned distanceIndex = 0;
      for(; distanceIndex < 4; distanceIndex++)
        if(lineIntersectionRelations.intersections[calcMLineMarker][distanceIndex] == calcCurrMarker)
          break;
      SAFE_ASSERT(distanceIndex < 4, static_cast<void>(0));

      for(unsigned i = 0; i < 4 && lineIntersectionRelations.intersections[calcMLineMarker][i] != MarkedIntersection::numOfIntersectionMarkers; i++)
        if(lineIntersectionRelations.intersections[calcMLineMarker][i] != calcCurrMarker)
        {
          const float distIntersections = lineIntersectionRelations.intersectionPositions[calcMLineMarker][distanceIndex]
                                          - lineIntersectionRelations.intersectionPositions[calcMLineMarker][i];

          for(unsigned intersectionIndex = 0; intersectionIndex < theFieldLineIntersections.intersections.size(); intersectionIndex++)
            if(!usedIntersection[intersectionIndex]
               && (theFieldLineIntersections.intersections[intersectionIndex].indexDir1 == mLine.lineIndex
                   || theFieldLineIntersections.intersections[intersectionIndex].indexDir2 == mLine.lineIndex)
               && std::abs((currIntersection.pos.y() < theFieldLineIntersections.intersections[intersectionIndex].pos.y() ? -1.f : 1.f) *
                           (currIntersection.pos - theFieldLineIntersections.intersections[intersectionIndex].pos).norm() - distIntersections) < 100.f) //FIXME
            {
              handleIntersections.emplace_back(intersectionIndex, mirror ?
                                               MarkedIntersection::mirror(lineIntersectionRelations.intersections[calcMLineMarker][i]) :
                                               lineIntersectionRelations.intersections[calcMLineMarker][i]);
              usedIntersection[intersectionIndex] = true;
            }
        }
    };

    if(usedLines[currIntersection.indexDir1])
    {
      //TODO assert last marker is same as this one
    }
    else
    {
      if(currentMI.marker >= MarkedIntersection::firstIntersectionMarkerOther)
        ff.markedLines.emplace_back(currIntersection.indexDir1, MarkedLine::mirror(intersectionLineRelations.lineMarkerOfDir1[currentMI.mirror()]));
      else
        ff.markedLines.emplace_back(currIntersection.indexDir1, intersectionLineRelations.lineMarkerOfDir1[currentMI.marker]);
      findIntersections(ff.markedLines.back()); //maybe do this everytime
    }

    if(usedLines[currIntersection.indexDir2])
    {
      //TODO assert last marker is same as this one
    }
    else
    {
      if(currentMI.marker >= MarkedIntersection::firstIntersectionMarkerOther)
        ff.markedLines.emplace_back(currIntersection.indexDir2, MarkedLine::mirror(intersectionLineRelations.lineMarkerOfDir2[currentMI.mirror()]));
      else
        ff.markedLines.emplace_back(currIntersection.indexDir2, intersectionLineRelations.lineMarkerOfDir2[currentMI.marker]);
      findIntersections(ff.markedLines.back());//maybe do this everytime
    }
  }
}

void IntersectionRelations::propagateMarkedLinePoint(const MarkedLine& start, const float linePoint, const Vector2f& fieldPoint,
    const FieldLineIntersections& theFieldLineIntersections, const FieldLines& theFieldLines, FieldFeature& ff) const
{
  const bool mirror = start.marker >= MarkedLine::firstLineMarkerOther;
  const MarkedLine::LineMarker calcMLineMarker = mirror ? start.mirror() : start.marker;

  for(unsigned i = 0; i < 4 && lineIntersectionRelations.intersections[calcMLineMarker][i] != MarkedIntersection::numOfIntersectionMarkers; i++)
  {
    const float distIntersections = linePoint - lineIntersectionRelations.intersectionPositions[calcMLineMarker][i];

    for(unsigned intersectionIndex = 0; intersectionIndex < theFieldLineIntersections.intersections.size(); intersectionIndex++)
      if((theFieldLineIntersections.intersections[intersectionIndex].indexDir1 == start.lineIndex
          || theFieldLineIntersections.intersections[intersectionIndex].indexDir2 == start.lineIndex)
         && std::abs((fieldPoint.y() < theFieldLineIntersections.intersections[intersectionIndex].pos.y() ? -1.f : 1.f) *
                     (fieldPoint - theFieldLineIntersections.intersections[intersectionIndex].pos).norm() - distIntersections) < 100.f) //FIXME
      {
        const MarkedIntersection foundIntersection(intersectionIndex, mirror ?
            MarkedIntersection::mirror(lineIntersectionRelations.intersections[calcMLineMarker][i]) :
            lineIntersectionRelations.intersections[calcMLineMarker][i]);
        return propagateMarkedIntersection(foundIntersection, theFieldLineIntersections, theFieldLines, ff);
      }
  }
  ff.markedLines.push_back(start);
}
