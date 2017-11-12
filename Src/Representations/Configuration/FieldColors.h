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

    numOfNonColors,

    ownJersey = numOfNonColors,
    opponentJersey,

    // new colors
  });

  STREAMABLE(BasicParameters,
  {
    BasicParameters(const unsigned char& mNCS, const unsigned char& bWD, const Rangeuc& fH);

    bool operator==(const BasicParameters& other) const { return maxNonColorSaturation == other.maxNonColorSaturation && blackWhiteDelimiter == other.blackWhiteDelimiter && fieldHue == other.fieldHue; }
    bool operator!=(const BasicParameters& other) const { return !(*this == other); }
    ,
    (unsigned char) maxNonColorSaturation,
    (unsigned char) blackWhiteDelimiter,
    (Rangeuc) fieldHue,
  });

  BasicParameters copyBasicParameters() { return BasicParameters(maxNonColorSaturation, blackWhiteDelimiter, fieldHue); }
  void setBasicParameters(const BasicParameters& p) { maxNonColorSaturation = p.maxNonColorSaturation; blackWhiteDelimiter = p.blackWhiteDelimiter; fieldHue = p.fieldHue; }

  Rangeuc& operator[](const Color c) { return colorHues[c - numOfNonColors]; }
  const Rangeuc& operator[](const Color c) const { return colorHues[c - numOfNonColors]; },

  (unsigned char)(64) maxNonColorSaturation,
  (unsigned char)(168) blackWhiteDelimiter,
  (Rangeuc) fieldHue,

  (Rangeuc[numOfColors - numOfNonColors]) colorHues,
});

inline FieldColors::BasicParameters::BasicParameters(const unsigned char& mNCS, const unsigned char& bWD, const Rangeuc& fH)
  : maxNonColorSaturation(mNCS), blackWhiteDelimiter(bWD), fieldHue(fH) {}
