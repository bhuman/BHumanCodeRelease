/**
 * @file WhistlePosition.h
 *
 * Identified whistle position
 *
 */

#pragma once

#include "Tools/Math/Eigen.h"

STREAMABLE(WhistlePosition,
{,
  (bool)(false) isValid, /**< Is the data in this representation currently valid? */
  (Vector2f) positionOnField, /**< Position relative to the center of the field. */
});
