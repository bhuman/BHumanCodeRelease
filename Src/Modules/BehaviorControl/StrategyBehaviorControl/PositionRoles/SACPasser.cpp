/**
 * @file SACPasser.cpp
 *
 * This file is an adaptation of the Passer role, for the Shared Autonomy Challenge.
 *
 * @author Yannik Meinken
 */

#include "SACPasser.h"
#include "Math/Geometry.h"

void SACPasser::preProcess()
{
  RatingRole::preProcess();
  MODIFY("behavior:Forward", p);

  //override parameters of the base class
  RatingRole::p.startThreshold = p.startThreshold;
  RatingRole::p.stopThreshold = p.stopThreshold;
}

float SACPasser::rating(const Vector2f& pos) const
{
  //outside the Voronoi region, the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //normal distribution around the base pose
  const float baseRating = std::exp(-0.5f * (pos - base).squaredNorm() / sqr(p.sigmaBase));
  const float passRating = thePassEvaluation.getRating(theFieldBall.recentBallPositionOnField(), pos, true);
  //const float goalRating = theExpectedGoals.getRating(pos, true);

  //get distance to next field border
  const float borderDistance = std::min(std::max(0.f, theFieldDimensions.xPosOpponentGoalLine - std::abs(pos.x())),
                                        std::max(0.f, theFieldDimensions.yPosLeftTouchline - std::abs(pos.y())));

  //better rating far away from the field border
  const float borderRating = 1.f - std::exp(-0.5f * sqr(borderDistance) / sqr(p.sigmaFieldBorder));

  // keep distance to region border
  float cellBorderRating = 1.f;
  Vector2f intersectionPoint;
  Geometry::Line intersectionLine;
  if(Geometry::getIntersectionOfLineAndConvexPolygon(region, Geometry::Line(base, pos - base), intersectionPoint, true, &intersectionLine))
  {
    const float distanceToCellBorder = Geometry::getDistanceToLineSigned(intersectionLine, pos);
    cellBorderRating = 1.f - std::exp(-0.5f * sqr(distanceToCellBorder) / sqr(p.sigmaCellBorder));
  }

  //do not stand directly near the ball as position role
  const float ballRating = 1.f - std::exp(-0.5f * (pos - theFieldBall.recentBallPositionOnField()).squaredNorm() / sqr(p.sigmaBall));

  //stay at the height of the ball
  const float ballHeightRating =  1 - std::exp(-std::abs(theFieldBall.recentBallPositionOnField().x() - theRobotPose.translation.x()) / (theFieldDimensions.xPosOpponentGoalLine * 2));

  //stay at a minimal distance from the teammate
  float teammateRating;
  for(const auto& teammate : theGlobalTeammatesModel.teammates)
  {
    if(teammate.playerNumber != theGameState.playerNumber)
    {
      teammateRating = 1.f / (1.f + (std::exp(-p.changeAtDistanceToTeammate * (std::abs((teammate.pose.translation - theRobotPose.translation).norm())) - p.minimalDistanceToTeammate)));
    }
  }

  // the focus is on the multiplication term because all ratings need to be good at the same time.
  // The additive term is for avoid getting stuck in large 0 areas where just one of the factors is 0. Should be deleted
  return cellBorderRating * borderRating * ballRating * passRating * baseRating * ballHeightRating * teammateRating * (1.f - p.addWeight) +
         (p.addWeight / 7) * (baseRating + passRating + ballRating + borderRating + cellBorderRating + ballHeightRating + teammateRating);
}
