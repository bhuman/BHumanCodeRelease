/**
* @file PenaltyAreaPerceptor.cpp
* Provides PenaltyArea.
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
*/

#include "PenaltyAreaPerceptor.h"
#include "Tools/Math/Geometry.h"
#include <vector>

void PenaltyAreaPerceptor::update(PenaltyArea& penaltyArea)
{
  penaltyArea.clear();
  penaltyArea.isValid = searchWithIntersections(penaltyArea);
}

bool PenaltyAreaPerceptor::searchWithIntersections(PenaltyArea& penaltyArea) const
{
  std::vector<const FieldLineIntersections::Intersection*> useFullIntersections;
  for(auto i = theFieldLineIntersections.intersections.begin(); i != theFieldLineIntersections.intersections.end(); i++)
    if(i->type != FieldLineIntersections::Intersection::X && i->additionalType == FieldLineIntersections::Intersection::none)
      useFullIntersections.push_back(&(*i));

  const float ttDistance = 2.f * theFieldDimensions.yPosLeftPenaltyArea;
  const float tlDistance = theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentPenaltyArea;
  const float halfPenaltyDepth = (theFieldDimensions.xPosOpponentGroundLine - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;

  for(auto i = useFullIntersections.begin(); i != useFullIntersections.end(); i++)
    for(auto j = i + 1; j != useFullIntersections.end(); j++)
      if((*i)->type == FieldLineIntersections::Intersection::T && (*j)->type == FieldLineIntersections::Intersection::T)
      {
        if(std::abs(((*i)->pos - (*j)->pos).norm() - ttDistance) < thresholdIntersections
           && std::abs(Angle((*i)->dir1.angle() - (*j)->dir1.angle()).normalize()) < thresholdAngleDisForIntersections)
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
          if(std::abs(Angle(tIntersection.dir1.angle() - lIntersection.dir1.angle() + pi).normalize()) < thresholdAngleDisForIntersections)
          {
            penaltyArea.rotation = (lIntersection.dir2 + tIntersection.dir2.rotated(pi)).angle() - 90_deg;
            penaltyArea.rotation.normalize();
            penaltyArea.translation = (lIntersection.pos + tIntersection.pos) / 2 + Vector2f(0.f, theFieldDimensions.yPosLeftPenaltyArea).rotate(penaltyArea.rotation);

            theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(tIntersection.ownIndex,
                ((penaltyArea.inverse() * tIntersection.pos).y() > 0.f) ? MarkedIntersection::STL : MarkedIntersection::STR),
                theFieldLineIntersections, theFieldLines, penaltyArea);
            return true;
          }
          else if(std::abs(Angle(tIntersection.dir1.angle() - lIntersection.dir2.angle() + pi).normalize()) < thresholdAngleDisForIntersections)
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

MAKE_MODULE(PenaltyAreaPerceptor, perception);
