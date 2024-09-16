/**
 * @file AttackingGoalkeeper.h
 *
 * This file declares the attacking goalkeeper role.
 *
 * @author Jonas Hotzan, Yannis Meyer
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "RatingRole.h"
#include "Tools/BehaviorControl/SectorWheel.h"

class AttackingGoalkeeper : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float)(0.01f) startThreshold, /**< the rating has to be at least this much better (normalized to the new found one) at the destination to start moving */
    (float)(0.05f) stopThreshold, /**< if the rating is not at least this much better (normalized to the new found one) at the destination stop moving */
    (float)(700.f) sigmaBaseX, /**< standard deviation for rating dependent on x-base position */
    (float)(2000.f) sigmaBaseY, /**< standard deviation for rating dependent on y-base position */
    (float)(0.25f) sigmaAngle, /**< standard deviation for rating the angle between ball and goalkeeper */
    (float)(400.f) sigmaOpponent, /**< standard deviation for rating the position behind the last opponent */
    (float)(300.f) sigmaTeammate, /**< standard deviation for rating the position in regard to the distance to a teammate */
    (float)(1000.f) sigmaCommunication, /**< standard deviation for rating dependent on the last target position */
    (float)(0.5f) minCommunicationRating, /**< minimal value of the rating depending on the last target position */
  });

  void preProcess() override;
  float rating(const Vector2f& pos) const override;
  float angleRating(const Vector2f& pointOnField) const;

  Parameters p;
};
