#include "GoalFramePerceptor.h"
#include "Tools/Math/Transformation.h"
#include <vector>

void GoalFramePerceptor::update(GoalFrame& goalFrame)
{
  goalFrame.clear();
  goalFrame.isGroundLineValid = false;

  if(searchByBigT(goalFrame) ||
     searchByBigX(goalFrame) ||
     searchByTT(goalFrame))
    goalFrame.isValid = goalFrame.isGroundLineValid = true;
  else
    goalFrame.isValid = false;

  // QUICK HACK RoboCup17 (Jesse)
  // Sometimes there are a few misdetections, but the benefit of searching them right now is not big enough.
  goalFrame.isValid = false;
}

bool GoalFramePerceptor::searchByBigT(GoalFrame& goalFrame) const
{
  std::vector<const IntersectionsPercept::Intersection*> useBigTIntersections;
  for(const IntersectionsPercept::Intersection& intersection : theIntersectionsPercept.intersections)
    if(intersection.type == IntersectionsPercept::Intersection::T &&
       (theLinesPercept.lines[intersection.line1Index].firstField - theLinesPercept.lines[intersection.line1Index].lastField).squaredNorm() > squaredBigIntersectionThreshold && //FIXME bad big calc for unseen corners
       (theLinesPercept.lines[intersection.line2Index].firstField - theLinesPercept.lines[intersection.line2Index].lastField).squaredNorm() > squaredBigIntersectionThreshold)
      useBigTIntersections.push_back(&intersection);

  for(const IntersectionsPercept::Intersection* intersection : useBigTIntersections)
  {
    const Pose2f prePose(intersection->dir1.angle() + pi_2, intersection->pos);
    if(GoalFramePerceptor::calcGoalFrame(prePose, theFieldDimensions.yPosLeftPenaltyArea, goalFrame))
      goalFrame.isGroundLineValid = true;
  }

  return false;
}

bool GoalFramePerceptor::searchByBigX(GoalFrame& goalFrame) const
{
  std::vector<const IntersectionsPercept::Intersection*> useBigXIntersections;
  for(const IntersectionsPercept::Intersection& intersection : theIntersectionsPercept.intersections)
    if(intersection.type == IntersectionsPercept::Intersection::X &&
       (theLinesPercept.lines[intersection.line1Index].firstField - theLinesPercept.lines[intersection.line1Index].lastField).squaredNorm() > squaredBigIntersectionThreshold && //FIXME bad big calc for unseen corners
       (theLinesPercept.lines[intersection.line2Index].firstField - theLinesPercept.lines[intersection.line2Index].lastField).squaredNorm() > squaredBigIntersectionThreshold)
      useBigXIntersections.push_back(&intersection);

  for(const IntersectionsPercept::Intersection* intersection : useBigXIntersections)
    if((intersection->pos - useBigXIntersections.front()->pos).squaredNorm() > sqrAllowedBXDivergence)
    {
      //ASSERT(false); //FIXME
      return false;
    }

  if(useBigXIntersections.empty())
    return false;

  const IntersectionsPercept::Intersection& bigX = *useBigXIntersections.front();

  const Pose2f prePose(bigX.dir1.angle() + pi_2, bigX.pos);

  return GoalFramePerceptor::calcGoalFrame(prePose, theFieldDimensions.yPosLeftPenaltyArea, goalFrame);
  //goalFrame.markedIntersections.push_back(MarkedIntersection(bigX, goalFrame.inverse().translation.y() > 0.f ? MarkedIntersection::STL : MarkedIntersection::STR));
}

bool GoalFramePerceptor::searchByTT(GoalFrame& goalFrame) const
{
  std::vector<const IntersectionsPercept::Intersection*> useTIntersections;
  for(const IntersectionsPercept::Intersection& intersection : theIntersectionsPercept.intersections)
    if(intersection.type == IntersectionsPercept::Intersection::T)
      useTIntersections.push_back(&intersection);

  static const float diffTT = theFieldDimensions.yPosLeftPenaltyArea - theFieldDimensions.yPosLeftGoal;
  static const float yTTPos = diffTT / 2.f + theFieldDimensions.yPosLeftGoal;

  for(unsigned i = 0u; i < useTIntersections.size(); i++)
    for(unsigned j = i + 1u; j < useTIntersections.size(); j++)
      if(std::abs((useTIntersections[i]->pos - useTIntersections[j]->pos).norm() - diffTT) < tTDistanceThreshold)
        if(Angle(useTIntersections[i]->dir1.angle()).diffAbs(useTIntersections[j]->dir1.angle()) < allowedTTAngleDivergence)
          if(calcGoalFrame(Pose2f(Angle(useTIntersections[i]->dir2.angle() + pi_2).normalize(),
                                  useTIntersections[i]->pos + 0.5f * (useTIntersections[j]->pos - useTIntersections[i]->pos)),
                           yTTPos, goalFrame))
            return true;

  return false;
}

bool GoalFramePerceptor::calcGoalFrame(const Pose2f& prePose, const float yTranslation, GoalFrame& goalFrame) const
{
  static const float borderToGroundLineDistance = theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOpponentGroundline;

  const Pose2f prePoseInverse(prePose.inverse());
  int positive = 0, negative = 0;
  for(const Vector2f& p : theFieldBoundary.boundaryOnField)
  {
    Vector2f pInField = prePoseInverse * p;
    if(std::abs(std::abs(pInField.x()) - borderToGroundLineDistance) < allowedFieldBoundaryDivergence)
    {
      if(pInField.x() > 0)
        positive++;
      else
        negative++;
    }
  }

  if(neededConvexBoundaryPoints > positive && neededConvexBoundaryPoints > negative)
    return false;

  const float sign = (positive < negative) ? -1.f : 1.f;
  goalFrame = Pose2f(prePose).rotate(-pi_2 + sign * pi_2);
  goalFrame.translation = goalFrame * Vector2f(0.f, (goalFrame.inverse().translation.y() > 0.f ? 1.f : -1.f)).normalized(yTranslation);
  return true;
}

MAKE_MODULE(GoalFramePerceptor, perception)
