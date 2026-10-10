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
    (float) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float) sigmaTeam, /**< standard deviation for rating dependent on distance to teammates */
    (float) sigmaBorder, /**< standard deviation for rating dependent on distance to field border */
    (float) ballDistance, /**< preferred distance to ball */
    (float) sigmaBall, /**< standard deviation for rating dependent on distance to ball */
    (float) minBallRating, /**< minimal value of the rating depending on the distance to the ball */
    (float) sigmaMark, /**< standard deviation for rating dependent on distance mark position */
    (float) minMarkRating, /**< minimal value of the mark rating */
    (float) distToMarkedRobot, /**< The minimum distance a robot must have to the marked robot. */
    (float) sigmaBallLine, /**< standard deviation for rating dependent on distance to the line from the ball to the goal */
    (float) minGoalLineRating, /**< minimal value of the rating dependent on the distance from the line between ball and goal */
    (float) sigmaCommunication, /**< standard deviation for rating dependent on the last target position */
    (float) minCommunicationRating, /**< minimal value of the rating depending on the last target position */
    (float) startThreshold, /**< the rating has to be at least this much better (normalized to the new found one) at the destination to start moving */
    (float) stopThreshold, /**< if the rating is not at least this much better (normalized to the new found one) at the destination stop moving */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;

  bool robotLeftOfLine = false;

public:
  Defender();
};
