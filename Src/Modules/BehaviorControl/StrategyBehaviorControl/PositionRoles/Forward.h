/**
 * @file Forward.h
 *
 * This file declares the forward role.
 * It tries to maximize a rating function in order to receive a pass and score a goal.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "RatingRole.h"
#include "Tools/BehaviorControl/KickSelection.h"

class Forward : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float) sigmaFieldBorder, /**< standard deviation for rating dependent on distance to field border */
    (float) sigmaCellBorder, /**< standard deviation for rating dependent on distance to cell border */
    (float) sigmaBall, /**< standard deviation for rating dependent on ball position. Do not stand directly near the ball as position role. */
    (float) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float) sigmaTeam, /**< standard deviation for rating dependent on distance to teammates */
    (float) sigmaCommunication, /**< standard deviation for rating dependent on the last target position */
    (float) minCommunicationRating, /**< minimal value of the rating depending on the last target position */
    (float) startThreshold, /**< the rating has to be at least this much better at the destination (normalized for the target pose) to start moving */
    (float) stopThreshold, /**< if the rating is not at least this much better at the destination (normalized for the target pose) stop moving */
    (float) addWeight, /**< how much should the individual rating functions be added additionally to the multiplication */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;

  bool robotLeftOfLine = false;

public:
  Forward();
};
