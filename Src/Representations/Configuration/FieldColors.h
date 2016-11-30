/**
* @file FieldColors.h
*
* Declaration of struct that holds the color classifications
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Range.h"

STREAMABLE(FieldColors,
{
  STREAMABLE(ColorRange,
  {
    bool operator==(const ColorRange & other) const
    {
      return y == other.y &&
      h == other.h &&
      s == other.s;
    }

    bool operator!=(const ColorRange & other) const
    {
      return !(*this == other);
    }
    ,
    (Rangeuc) y,
    (Rangeuc) h,
    (Rangeuc) s,
  });

  ENUM(Color,
  {,
    none,
    white,
    black,
    numOfNonColors,

    green = numOfNonColors,
    
    ownJersey,
    opponentJersey,

    // new colors
  });

  ColorRange& operator[](const Color c) { return colorRanges[c - numOfNonColors]; }
  const ColorRange& operator[](const Color c) const { return colorRanges[c - numOfNonColors]; }
  ,
  (unsigned char)(175) minYWhite,
  (unsigned char)(40) maxYBlack,
  (unsigned char)(64) maxNonColorSaturation,
  (ColorRange[numOfColors - numOfNonColors]) colorRanges,
});
