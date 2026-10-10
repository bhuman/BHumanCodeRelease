/**
 * @file Midfielder.h
 *
 * This file declares the Midfielder role.
 * It tries to maximize a rating function.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "RatingRole.h"
#include "Tools/BehaviorControl/KickSelection.h"

class Midfielder : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float) sigmaTeam, /**< standard deviation for rating dependent on distance to teammates */
    (float) sigmaFieldBorder, /**< standard deviation for rating dependent on distance to field border */
    (float) sigmaCellBorder, /**< standard deviation for rating dependent on distance to cell border */
    (float) ballDistance, /**< preferred distance to ball */
    (float) sigmaBall, /**< standard deviation for rating dependent on distance to ball */
    (float) startThreshold, /**< the rating has to be at least this much better (normalized to the new found one) at the destination to start moving */
    (float) stopThreshold, /**< if the rating is not at least this much better (normalized to the new found one) at the destination stop moving */
    (float) baseShiftX, /**< factor by which the base pose is moved toward the X position of the ball */
    (float) maxBaseShiftY, /**< factor by which the base pose is moved toward the Y position of the ball at maximum */
    (float) smoothBaseShiftYDistance, /**< distance from the mid line in which the base shift in Y direction is smoothed linearly */
    (float) minBallRating, /**< minimal value of the rating depending on the distance to the ball */
    (float) minGoalRating, /**< minimal value of the goal rating */
    (float) sigmaCommunication, /**< standard deviation for rating dependent on the last target position */
    (float) minCommunicationRating, /**< minimal value of the rating depending on the last target position */
    (float) minNotAllowedBallDistance, /**< The distance to the ball that is not allowed. */
    (float) ballBorderDistanceRatio, /**< Use the distance between ball and the baseArea to ensure, there is always area for a good ball position. */
    (float) minBallNotAllowedDistance, /**< Minimal interpolation rating very close to the ball. */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;

  bool robotLeftOfLine = false;

  bool calcBall = true;
  bool calcBase = true;
  bool calcPass = true;
  bool calcCellBorder = true;
  float maxBallDistanceToBorderPoints = 0.f;

public:
  Midfielder();
};
