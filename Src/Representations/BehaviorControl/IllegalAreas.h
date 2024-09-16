/**
 * @file IllegalAreas.h
 *
 * This file declares a representation that describes which field areas are illegal to enter.
 *
 * @author Arne Hasselbring
 * @author Fynn Böse
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include "Streaming/Function.h"

STREAMABLE(IllegalAreas,
{
  ENUM(FieldArea,
  {,
    ownGoalArea, /**< The own goal area (including its bounding lines). */
    ownPenaltyArea, /**< The own penalty area (including its bounding lines). */
    opponentPenaltyArea, /**< The opponent penalty area (including its bounding lines). */
    borderStrip, /**< The border strip (excluding the border lines). */
    opponentHalf, /**< The opponent half (including the halfway line, but without the center circle and its perimeter). */
    centerCircle, /**< The center circle (including its perimeter). */
    ballArea, /**< The 0.75m area around the ball. */
    notOwnGoalLine, /**< Everything except for the own goal line (for goalkeeper during penalty kick). */
  });

  /**
   * Get bitset of \c FieldArea constants that are illegal to enter for this player and the position is part of.
   * @param positionOnField A position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @return Bitset of \c FieldArea constants that are illegal to enter for this player and the position is part of.
   */
  FUNCTION(unsigned(const Vector2f& positionOnField, float margin)) getIllegalAreas;

  /**
   * Checks whether a given position on the field is illegal.
   * @param positionOnField A position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @return Whether the position is illegal.
   */
  FUNCTION(bool(const Vector2f& positionOnField, float margin)) isPositionIllegal;

  /**
   * Checks whether a given position will be illegal at the end of the current game state or set play.
   * So far, this only works if the current game state is ready or an in-game penalty kick takes place.
   * @param positionOnField A position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @return Whether the position will be illegal.
   */
  FUNCTION(bool(const Vector2f& positionOnField, float margin)) willPositionBeIllegal;

  /**
   * Checks whether a given position will be illegal in the given period of time.
   * @param positionOnField A position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @param duration A period of time to check if the current position will be illegal in that time.
   * @return Whether the position will be illegal in the given period of time.
   */
  FUNCTION(bool(const Vector2f& positionOnField, float margin, int duration)) willPositionBeIllegalIn;

  /**
   * Checks whether two given positions on the field are within the same illegal area.
   * @param positionOnField First position to check (assumed to be a point with no spatial extent).
   * @param targetOnField Second position to check (assumed to be a point with no spatial extent).*
   * @param margin A positive number extends the illegal areas.
   * @return Whether the two positions are within the same illegal area.
   */
  FUNCTION(bool(const Vector2f& positionOnField, const Vector2f& targetOnField, float margin)) isSameIllegalArea,

  (unsigned)(0u) illegal, /**< Bitset of \c FieldArea constants that are illegal to enter for this player. */
  (unsigned)(0u) anticipatedIllegal, /**< Bitset of \c FieldArea constants that illegal for this player in the future. */
  (unsigned)(0u) anticipatedTimestamp, /**< Timestamp when \c anticipatedIllegal comes into effect. */
});
