/**
 * @file IllegalAreaProvider.h
 *
 * This file declares a module that determines which field areas are illegal to enter.
 *
 * @author Arne Hasselbring
 * @author Fynn Böse
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/Libraries/LibTeammates.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"

MODULE(IllegalAreaProvider,
{,
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(LibTeammates),
  REQUIRES(StrategyStatus),
  REQUIRES(FrameInfo),
  PROVIDES(IllegalAreas),
  DEFINES_PARAMETERS(
  {,
    (float)(750.f) freeKickClearAreaRadius, /**< The radius of the area that has to be cleared by the defending team during a free kick. */
  }),
});

class IllegalAreaProvider : public IllegalAreaProviderBase
{
  /**
   * This method updates the illegal areas.
   * @param illegalAreas The updated representation.
   */
  void update(IllegalAreas& illegalAreas) override;

  /**
   * Get bitset of \c FieldArea constants that are illegal to enter for this player and the position is part of.
   * @param positionOnField A position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @return Bitset of \c FieldArea constants that are illegal to enter for this player and the position is part of.
   */
  unsigned getIllegalAreas(unsigned illegal, const Vector2f& positionOnField, float margin) const;

  /**
   * Checks whether a given position on the field is illegal.
   * @param illegal A mask describing which areas are actually illegal.
   * @param positionOnField A position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @return Whether the position is illegal.
   */
  bool isPositionIllegal(unsigned illegal, const Vector2f& positionOnField, float margin) const;

  /**
   * Checks whether two given positions on the field are within the same illegal area.
   * @param illegal A mask describing which areas are actually illegal.
   * @param positionOnField First position to check (assumed to be a point with no spatial extent).
   * @param targetOnField Second position to check (assumed to be a point with no spatial extent).
   * @param margin A positive number extends the illegal areas.
   * @return Whether the two positions are within the same illegal area.
   */
  bool isSameIllegalArea(unsigned illegal, const Vector2f& positionOnField, const Vector2f& targetOnField, float margin) const;

  /**
   * Visualizes the most common illegal areas with debug drawings.
   * @param illegal A mask describing which areas are actually illegal.
   */
  void draw(unsigned illegal, unsigned anticipatedIllegal) const;
};
