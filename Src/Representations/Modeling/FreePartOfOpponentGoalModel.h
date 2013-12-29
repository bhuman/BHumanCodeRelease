/**
 * @file Representations/Modeling/FreePartOfOpponentGoalModel.h
 * Declaration of a representation that represents the free part of the opponent goal
 * @author Carsten Könemann
 */

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FreePartOfOpponentGoalModel,
{
public:
  /** Draws the free part of goal to the field view */
  void draw() const,

  (Vector2<>) leftEnd, /**< relative position on field of left end of largest free part of opponent goal */
  (Vector2<>) rightEnd, /**< relative position on field of right end of largest free part of opponent goal */
  (bool)(false) valid, /**< if there is currently a reasonable part of the opponent goal free */
});
