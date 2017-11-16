#include "GoalFeaturePerceptor.h"
#include "Tools/Math/Geometry.h"
#include <vector>

void GoalFeaturePerceptor::update(GoalFeature& goalFeature)
{
  goalFeature.clear();
  goalFeature.isValid = searchByGoalPostAndT(goalFeature);
  theLastGoalPostPercept = theGoalPostPercept;
}

bool GoalFeaturePerceptor::searchByGoalPostAndT(GoalFeature& goalFeature) const
{
  if(!theGoalPostPercept.wasSeen && !theLastGoalPostPercept.wasSeen)
    return false;

  const Vector2f goalPostPosition = theGoalPostPercept.wasSeen ?
                                    theGoalPostPercept.positionOnField : theOdometer.odometryOffset * theLastGoalPostPercept.positionOnField;

  const float fieldDistance = theFieldDimensions.yPosLeftPenaltyArea - theFieldDimensions.yPosLeftGoal;

  std::vector<const FieldLineIntersections::Intersection*> useNormalTIntersections;
  for(const FieldLineIntersections::Intersection& intersection : theFieldLineIntersections.intersections)
    if(intersection.type == FieldLineIntersections::Intersection::T
       && intersection.additionalType == FieldLineIntersections::Intersection::none
       && std::abs((intersection.pos - goalPostPosition).norm() - fieldDistance) < thresholdMaxDisVaranzGoalPostT)
      useNormalTIntersections.push_back(&intersection);

  for(const FieldLineIntersections::Intersection* intersection : useNormalTIntersections)
  {
    const Pose2f intersectionPose(intersection->dir1.angle(), intersection->pos);
    const Vector2f postInIntersection = intersectionPose.inverse() * goalPostPosition;

    if(std::abs(postInIntersection.x()) > thresholdMaxXVaranzGoalPostT)
      continue;

    const bool isLeft = postInIntersection.y() > 0;
    goalFeature.translation = goalPostPosition + intersection->dir2.normalized(isLeft ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);
    goalFeature.rotation = Angle(pi + intersectionPose.rotation).normalize();

    theIntersectionRelations.propagateMarkedIntersection(
      MarkedIntersection(intersection->ownIndex, isLeft ? MarkedIntersection::STL : MarkedIntersection::STR),
      theFieldLineIntersections, theFieldLines, goalFeature);
    goalFeature.markedPoints.emplace_back(goalPostPosition, isLeft ? MarkedPoint::goalPostL : MarkedPoint::goalPostR, theGoalPostPercept.wasSeen);
    return true;
  }

  return false;
}

MAKE_MODULE(GoalFeaturePerceptor, perception)
