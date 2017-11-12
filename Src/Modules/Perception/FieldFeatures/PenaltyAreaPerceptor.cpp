#include "PenaltyAreaPerceptor.h"
#include "Tools/Math/Geometry.h"
#include <vector>

void PenaltyAreaPerceptor::update(PenaltyArea& penaltyArea)
{
  penaltyArea.clear();

  if(searchWithPMarkAndLine(penaltyArea) ||
     searchWithIntersections(penaltyArea))
    penaltyArea.isValid = true;
  else
    penaltyArea.isValid = false;

  lastFrameTime = theFrameInfo.time;
  theLastPenaltyMarkPercept = thePenaltyMarkPercept;
}

bool PenaltyAreaPerceptor::searchWithPMarkAndLine(PenaltyArea& penaltyArea) const
{
  if(!usePenaltyMark
     || !(thePenaltyMarkPercept.wasSeen
          || (theLastPenaltyMarkPercept.wasSeen
              && theFrameInfo.getTimeSince(lastFrameTime) <= maxTimeOffset // because of log forwards jumps (and missing images)
              && theFrameInfo.time >= lastFrameTime) // because of logs backwards jumps
         ))
    return false;

  const Vector2f thePenaltyMarkPosition = thePenaltyMarkPercept.wasSeen ? thePenaltyMarkPercept.positionOnField : theOdometer.odometryOffset * theLastPenaltyMarkPercept.positionOnField;

  static const float disPenaltyMarkGroundLine = theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyMark;
  static const float disPenaltyMarkPenaltyArea = theFieldDimensions.xPosOpponentPenaltyArea - theFieldDimensions.xPosOpponentPenaltyMark;

  for(unsigned i = 0; i < theFieldLines.lines.size(); i++)
  {
    const Geometry::Line geomLine(theFieldLines.lines[i].first, theFieldLines.lines[i].last - theFieldLines.lines[i].first);
    const float rawDistanceToLine = Geometry::getDistanceToLine(geomLine, thePenaltyMarkPosition);
    const float distanceToLine = std::abs(rawDistanceToLine);
    const float halfLength = (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;

    if(theFieldLines.lines[i].isLong && std::abs(distanceToLine - disPenaltyMarkGroundLine) < thresholdDisVaranzToPenaltyMark)
    {
      const Vector2f offset = geomLine.direction.normalized(distanceToLine - halfLength).rotate(rawDistanceToLine > 0 ? pi_2 : -pi_2);
      const Pose2f posePA(offset.angle(), thePenaltyMarkPosition + offset);

      if(!checkLineDistanceToNearesPoint(posePA, theFieldLines.lines[i].first, theFieldLines.lines[i].last))
        continue;

      penaltyArea = posePA;

      penaltyArea.markedPoints.emplace_back(thePenaltyMarkPosition, MarkedPoint::penaltyMark, thePenaltyMarkPercept.wasSeen);
      theIntersectionRelations.propagateMarkedLinePoint(MarkedLine(i, MarkedLine::groundLine), 0.f, Pose2f(penaltyArea.rotation, thePenaltyMarkPosition) * Vector2f(distanceToLine, 0.f),
          theFieldLineIntersections, theFieldLines, penaltyArea);

      return true;
    }
    else if(std::abs(distanceToLine - disPenaltyMarkPenaltyArea) < thresholdDisVaranzToPenaltyMark)
    {
      const Vector2f offset = geomLine.direction.normalized(distanceToLine + halfLength).rotate(rawDistanceToLine > 0 ? pi_2 : -pi_2);
      const Pose2f posePA(offset.angle(), thePenaltyMarkPosition + offset);

      if(!checkLineDistanceToNearesPoint(posePA, theFieldLines.lines[i].first, theFieldLines.lines[i].last))
        continue;

      penaltyArea = posePA;

      penaltyArea.markedPoints.emplace_back(thePenaltyMarkPosition, MarkedPoint::penaltyMark, thePenaltyMarkPercept.wasSeen);
      theIntersectionRelations.propagateMarkedLinePoint(MarkedLine(i, MarkedLine::groundPenalty), 0.f, Pose2f(penaltyArea.rotation, thePenaltyMarkPosition) * Vector2f(distanceToLine, 0.f),
          theFieldLineIntersections, theFieldLines, penaltyArea);

      return true;
    }
  }
  return false;
}

bool PenaltyAreaPerceptor::searchWithIntersections(PenaltyArea& penaltyArea) const
{
  std::vector<const FieldLineIntersections::Intersection*> useFullIntersections;
  for(auto i = theFieldLineIntersections.intersections.begin(); i != theFieldLineIntersections.intersections.end(); i++)
    if(i->type != FieldLineIntersections::Intersection::X && i->additionalType == FieldLineIntersections::Intersection::none)
      useFullIntersections.push_back(&(*i));

  const float ttDistance = 2.f * theFieldDimensions.yPosLeftPenaltyArea;
  const float tlDistance = theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea;
  const float halfPenaltyDepth = (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;

  for(auto i = useFullIntersections.begin(); i != useFullIntersections.end(); i++)
    for(auto j = i + 1; j != useFullIntersections.end(); j++)
      if((*i)->type == FieldLineIntersections::Intersection::T && (*j)->type == FieldLineIntersections::Intersection::T)
      {
        if(std::abs(((*i)->pos - (*j)->pos).norm() - ttDistance) < thresholdIntersections
           && std::abs(Angle((*i)->dir1.angle() - (*j)->dir1.angle()).normalize()) < thesholdAngleDisForIntersections)
        {
          penaltyArea.rotation = ((*i)->dir2 + (*i)->dir2).angle() + 90_deg;
          penaltyArea.rotation.normalize();
          penaltyArea.translation = ((*i)->pos + (*j)->pos) / 2.f - Vector2f(halfPenaltyDepth, 0.f).rotate(penaltyArea.rotation);

          theIntersectionRelations.propagateMarkedIntersection(
            MarkedIntersection((*i)->ownIndex, ((penaltyArea.inverse() * (*i)->pos).y() > 0.f) ? MarkedIntersection::STR : MarkedIntersection::STL),
            theFieldLineIntersections, theFieldLines, penaltyArea);
          return true;
        }
      }
      else if((*i)->type != FieldLineIntersections::Intersection::L || (*j)->type != FieldLineIntersections::Intersection::L)
      {
        const FieldLineIntersections::Intersection& tIntersection = ((*i)->type == FieldLineIntersections::Intersection::L) ? **j :** i;
        const FieldLineIntersections::Intersection& lIntersection = ((*i)->type == FieldLineIntersections::Intersection::L) ? **i :** j;
        if(std::abs((tIntersection.pos - lIntersection.pos).norm() - tlDistance) < thresholdIntersections &&
           std::abs((Pose2f(tIntersection.dir1.angle(), tIntersection.pos).inverse() * lIntersection.pos).y()) < thresholdYVarianceIntersection)
        {
          if(std::abs(Angle(tIntersection.dir1.angle() - lIntersection.dir1.angle() + pi).normalize()) < thesholdAngleDisForIntersections)
          {
            penaltyArea.rotation = (lIntersection.dir2 + tIntersection.dir2.rotated(pi)).angle() - 90_deg;
            penaltyArea.rotation.normalize();
            penaltyArea.translation = (lIntersection.pos + tIntersection.pos) / 2 + Vector2f(0.f, theFieldDimensions.yPosLeftPenaltyArea).rotate(penaltyArea.rotation);

            theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(tIntersection.ownIndex,
                ((penaltyArea.inverse() * tIntersection.pos).y() > 0.f) ? MarkedIntersection::STL : MarkedIntersection::STR),
                theFieldLineIntersections, theFieldLines, penaltyArea);
            return true;
          }
          else if(std::abs(Angle(tIntersection.dir1.angle() - lIntersection.dir2.angle() + pi).normalize()) < thesholdAngleDisForIntersections)
          {
            penaltyArea.rotation = (lIntersection.dir1 + tIntersection.dir2).angle() + 90_deg;
            penaltyArea.rotation.normalize();
            penaltyArea.translation = (lIntersection.pos + tIntersection.pos) / 2 - Vector2f(0.f, theFieldDimensions.yPosLeftPenaltyArea).rotate(penaltyArea.rotation);

            theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(tIntersection.ownIndex,
                ((penaltyArea.inverse() * tIntersection.pos).y() > 0.f) ? MarkedIntersection::STL : MarkedIntersection::STR),
                theFieldLineIntersections, theFieldLines, penaltyArea);
            return true;
          }
        }
      }

  return false;
}

bool PenaltyAreaPerceptor::checkLineDistanceToNearesPoint(const Pose2f& posePA, const Vector2f& p1, const Vector2f& p2) const
{
  const Pose2f invPosePA = posePA.inverse();
  const Vector2f p1InPA = invPosePA * p1;
  const Vector2f p2InPA = invPosePA * p2;

  return p1InPA.y() * p2InPA.y() <= 0.f
         || std::abs(p1InPA.y()) < maxDistVarianceOfLineEnds
         || std::abs(p2InPA.y()) < maxDistVarianceOfLineEnds;
}

MAKE_MODULE(PenaltyAreaPerceptor, perception)
