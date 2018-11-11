/**
 * @file FieldColors.h
 *
 * Declaration of struct that holds the color classifications.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Range.h"

STREAMABLE(FieldColors,
{
  ENUM(Color,
  {,
    none,
    white,
    black,
    field,
  });

  bool operator==(const FieldColors& other) const
  {
    return maxNonColorSaturation == other.maxNonColorSaturation
           && blackWhiteDelimiter == other.blackWhiteDelimiter
           && fieldHue == other.fieldHue;
  }

  bool operator!=(const FieldColors& other) const {return !(*this == other);},

  (unsigned char)(64) maxNonColorSaturation,
  (unsigned char)(168) blackWhiteDelimiter,
  (Rangeuc) fieldHue,
});
