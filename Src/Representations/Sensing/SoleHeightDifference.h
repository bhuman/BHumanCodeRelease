/**
 * @file SoleHeightDifference.h
 *
 * This file defines a representation that describes the height difference between both soles.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Legs.h"

STREAMABLE(SoleHeightDifference,
{,
  (float)(0.f) difference, /**< Positive -> right sole is higher, Negative -> left sole is higher. Difference in mm. */
});
