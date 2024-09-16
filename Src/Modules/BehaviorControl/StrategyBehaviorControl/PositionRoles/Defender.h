/**
 * @file Defender.h
 *
 * This file declares the Defender role.
 * It tries to maximize a rating function.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "RatingRole.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Math/RingBufferWithSum.h"

class Defender : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float)(2000.f) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float)(1000.f) sigmaTeam, /**< standard deviation for rating dependent on distance to teammates */
    (float)(600.f) sigmaBorder, /**< standard deviation for rating dependent on distance to field border */
    (float)(500.f) sigmaMark, /**< standard deviation for rating dependent on distance mark position */
    (float)(0.2f) minMarkRating, /**< minimal value of the mark rating */
    (float)(750.f) distToMarkedRobot, /**< The minimum distance a robot must have to the marked robot. */
    (float)(750.f) sigmaBallLine, /**< standard deviation for rating dependent on distance to the line from the ball to the goal */
    (float)(0.2f) minGoalLineRating, /**< minimal value of the rating dependent on the distance from the line between ball and goal */
    (float)(1000.f) sigmaCommunication, /**< standard deviation for rating dependent on the last target position */
    (float)(0.5f) minCommunicationRating, /**< minimal value of the rating depending on the last target position */
    (float)(0.3f) startThreshold, /**< the rating has to be at least this much better (normalized to the new found one) at the destination to start moving */
    (float)(0.1f) stopThreshold, /**< if the rating is not at least this much better (normalized to the new found one) at the destination stop moving */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;
};
