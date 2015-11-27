/**
 * @author: marcel
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/ColorClasses.h"
#include "Tools/Range.h"

STREAMABLE(ColorCalibration,
{
  /**
   * This struct describes the thresholds for white.
   * Thresholds are inclusive.
   */
  STREAMABLE(WhiteThresholds,
  {
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

    (Rangei)(0) hue,
    (Rangei)(0) saturation,
    (Rangei)(0) intensity,
  }),

  // thresholds for color
  (WhiteThresholds) white,
  (HSIRanges[ColorClasses::numOfColors - 2]) ranges,
});
