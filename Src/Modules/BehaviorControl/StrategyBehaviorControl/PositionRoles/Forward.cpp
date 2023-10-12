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
  MODIFY("parameters:behavior:Forward", p);

  //override parameters of the base class
  RatingRole::p.startThreshold = p.startThreshold;
  RatingRole::p.stopThreshold = p.stopThreshold;
}

float Forward::rating(const Vector2f& pos) const
{
  //outside the voronoi region the rating is 0
  if(!Geometry::isPointInsideConvexPolygon(region.data(), static_cast<int>(region.size()), pos))
    return 0.f;

  //normal distribution around the base pose
  const float baseRating = std::exp(-0.5f * (pos - base).squaredNorm() / sqr(p.sigmaBase));

  const float passRating = thePassEvaluation.getRating(pos);
  const float goalRating = theExpectedGoals.getRating(pos);

  //the focus is on the multiplication term because all ratings need to be good at the same time. The additive term is for avoid getting stuck in large 0 areas where just one of the factors is 0. Sxhould be deleted
  return passRating * goalRating * baseRating * (1.f - p.addWeight) + (p.addWeight / 3) * (baseRating + goalRating + passRating);
}
