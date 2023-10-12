/**
 * @file Midfielder.cpp
 *
 * This file implements the Midfielder role.
 * Tries to cover as much space as possible by staying far away from the teammates and the field border while not deviate to much from the base pose.
 * It tries to maximize a rating function that is manly based on the distance to the closest teammate.
 *
 * @author Yannik Meinken
 */

#include "Midfielder.h"
#include "Math/Geometry.h"

void Midfielder::preProcess()
{
  RatingRole::preProcess();
  DECLARE_DEBUG_DRAWING("behavior:Midfielder:shiftedPosition", "drawingOnField");
  MODIFY("parameters:behavior:Midfielder", p);

  //override parameters of the base class
  RatingRole::p.startThreshold = p.startThreshold;
  RatingRole::p.stopThreshold = p.stopThreshold;
}

float Midfielder::rating(const Vector2f& pos) const
{
  //outside the voronoi region the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  Vector2f shiftedBase = base;
  // if the ball is in the opponent half shift position towards ball to quickly respond to a counter-attack
  if(ball.x() > theFieldDimensions.xPosHalfWayLine && !(theGameState.isFreeKick() && theGameState.isForOwnTeam()))
  {
    shiftedBase.x() = shiftedBase.x() + ball.x() * p.baseShiftX;
    //shift the y coordinate depending on how far the ball is in the opponent half to avoid a jump at the border that would need hysteresis
    const float yFactor = std::min(p.maxBaseShilftY, ball.x() / p.smoothBaseShiftYDistence);
    shiftedBase.y() = shiftedBase.y() + (ball.y() - shiftedBase.y()) * yFactor;

    //if position is outside the voronoi region clip it
    Geometry::clipPointInsideConvexPolygon(region, shiftedBase);
    CROSS("behavior:Midfielder:shiftedPosition", shiftedBase.x(), shiftedBase.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);
  }

  //normal distribution around the shifted base pose
  const float baseRating = std::exp(-0.5f * (pos - shiftedBase).squaredNorm() / sqr(p.sigmaBase));

  //min value with a compressed normal distribution on top
  const float ballRating = std::exp(-0.5f * sqr((pos - ball).norm() - p.ballDistance) / sqr(p.sigmaBall)) * (1.f - p.minBallRating) + p.minBallRating;

  //get distance to next field border
  const float borderDistance = std::min(std::max(0.f, theFieldDimensions.xPosOpponentGroundLine - std::abs(pos.x())),
                                        std::max(0.f, theFieldDimensions.yPosLeftSideline - std::abs(pos.y())));

  //better rating far away from the field border
  const float borderRating = 1.f - std::exp(-0.5f * sqr(borderDistance) / sqr(p.sigmaBorder));

  //get the rating based on the nearest teammate
  float rating = 1;
  if(!theGlobalTeammatesModel.teammates.empty())
  {
    float minTeammateDistanceSquared = (pos - theGlobalTeammatesModel.teammates.cbegin()->pose.translation).squaredNorm();
    for(auto t = ++theGlobalTeammatesModel.teammates.cbegin(); t != theGlobalTeammatesModel.teammates.cend(); t++)
    {
      const float d = (pos - t->pose.translation).squaredNorm();
      if(d < minTeammateDistanceSquared)
        minTeammateDistanceSquared = d;
    }

    //better rating far away from teammates
    rating = 1.f - std::exp(-0.5f * minTeammateDistanceSquared / sqr(p.sigmaTeam));
  }

  //in case of a free kick we know that we are the attacking team so we can play offensively
  //search for free spaces to receive a pass
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
    return baseRating * thePassEvaluation.getRating(pos) * (p.minGoalRating + (1.f - p.minGoalRating) * theExpectedGoals.getRating(pos));

  //combine the ratings
  return ballRating * rating * baseRating * borderRating;
}
