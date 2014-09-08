/*
 * @author: marcel
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/ColorClasses.h"
#include "Tools/Range.h"

STREAMABLE(ColorCalibration,
{
public:
  /**
   * This class describes the thresholds for white.
   * Thresholds are inclusive.
   */
  STREAMABLE(WhiteThresholds,
  {
  public:
    bool operator==(const WhiteThresholds& other) const
    {
      return minR == other.minR &&
             minB == other.minB &&
             minRB == other.minRB;
    }

    bool operator!=(const WhiteThresholds& other) const
    {
      return !(*this == other);
    },

    (int)(0) minR,
    (int)(0) minB,
    (int)(0) minRB,
  });

  /**
   * Defines a full configuration of a color. All values are inclusive.
   */
  STREAMABLE(HSIRanges,
  {
  public:
    bool operator==(const HSIRanges& other) const
    {
      return hue == other.hue &&
             saturation == other.saturation &&
             intensity == other.intensity;
    }

    bool operator!=(const HSIRanges& other) const
    {
      return !(*this == other);
    },

    (Range<int>)(0) hue,
    (Range<int>)(0) saturation,
    (Range<int>)(0) intensity,
  }),

  // thresholds for color
  (WhiteThresholds) white,
  (HSIRanges[ColorClasses::numOfColors - 2]) ranges,
});
