#include "MidCornerPerceptor.h"

void MidCornerPerceptor::update(MidCorner& midCorner)
{
  midCorner.clear();

  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    midCorner.isValid = false;
    return;
  }

  if(searchForBigT(midCorner))
    midCorner.isValid = true;
  else
    midCorner.isValid = false;
}

bool MidCornerPerceptor::searchForBigT(MidCorner& midCorner) const
{
  for(auto& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::T &&
       (intersection.additionalType == FieldLineIntersections::Intersection::big || intersection.additionalType == FieldLineIntersections::Intersection::mid))
    {
      midCorner.translation = intersection.pos;
      midCorner.rotation = intersection.dir1.angle();

      midCorner.isValid = true; //< allows the next line to calculate
      if(!(midCorner.isValid = theFieldDimensions.isInsideCarpet(midCorner.getGlobalFeaturePosition() * midCorner.inverse().translation)))
        continue;

      midCorner.clear();
      theIntersectionRelations.propagateMarkedIntersection(MarkedIntersection(intersection.ownIndex,  MarkedIntersection::BT),
          theFieldLineIntersections, theFieldLines, midCorner);

      if(!midCorner.isLikeEnoughACorrectPerception(distrustAreaXRadius, distrustAreaYRadius, theFieldLines, distrustAreaOffset))
        continue;

      return true;
    }
  return false;
}

MAKE_MODULE(MidCornerPerceptor, perception)
