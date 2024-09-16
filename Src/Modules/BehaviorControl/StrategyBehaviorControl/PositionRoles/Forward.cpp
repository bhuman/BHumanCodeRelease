/**
 * @file Forward.cpp
 *
 * This file implements the forward role.
 * It tries to maximize a rating function in order to receive a pass and score a goal.
 *
 * @author Yannik Meinken
 */

#include "Forward.h"
#include "Math/Geometry.h"

void Forward::preProcess()
{
  RatingRole::preProcess();
  DECLARE_DEBUG_DRAWING("behavior:Forward:communicatedPosition", "drawingOnField");
  MODIFY("behavior:Forward", p);

  //override parameters of the base class
  RatingRole::p.startThreshold = p.startThreshold;
  RatingRole::p.stopThreshold = p.stopThreshold;
}

float Forward::rating(const Vector2f& pos) const
{
  //outside the Voronoi region, the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //normal distribution around the base pose
  const float baseRating = std::exp(-0.5f * (pos - base).squaredNorm() / sqr(p.sigmaBase));
  const float passRating = thePassEvaluation.getRating(theFieldBall.recentBallPositionOnField(), pos, true);
  const float goalRating = theExpectedGoals.getRating(pos, true);

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

  // Positions near the last communicated target pose are better
  const Vector2f lastTargetInWorld = agent.lastKnownPose * agent.lastKnownTarget; // Transform from relative to global Coordinates
  const float communicationRating = p.minCommunicationRating +
      (1.f - p.minCommunicationRating) * std::exp(-0.5f * (pos - lastTargetInWorld).squaredNorm() / sqr(p.sigmaCommunication));
  CROSS("behavior:Forward:communicatedPosition", lastTargetInWorld.x(), lastTargetInWorld.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);

  //the focus is on the multiplication term because all ratings need to be good at the same time.
  // The additive term is for avoid getting stuck in large 0 areas where just one of the factors is 0. Should be deleted
  return communicationRating * cellBorderRating * borderRating * ballRating * passRating * goalRating * baseRating * (1.f - p.addWeight) +
         (p.addWeight / 7) * (baseRating + goalRating + passRating + ballRating + borderRating + cellBorderRating + communicationRating);
}
