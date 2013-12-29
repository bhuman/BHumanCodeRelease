#pragma once

#include "Tools/Range.h"
#include "Tools/Debugging/DebugDrawings.h"

class DrawingHelpers{
public:
  /**
   * Generates a color for a heat map or similar debug drawings. High values will
   * be displayed as red and low values as blue.
   * @param value The value that should be displayed.
   * @param minValue The minimal value value could be.
   * @param maxValue The maximal value value could be.
   * @param alpha The alpha channel of the result.
   * @param overlappingScale High values will result in clearly seperated colors.
   *                         Values between 1 and 15 are recommended.
   */
  static ColorRGBA heat(float value, float maxValue, float minValue = 0.0, float overlappingScale = 5.0f, unsigned char alpha = 255)
  {
    // see gnuplot: plot 1-2*x**2, 1-2*(x-0.5)**2, 1-2*(x-1)**2
    value -= minValue;
    if(maxValue == 0.0f)
      maxValue = value;
    const Range<> range(0.0f, 1.0f); // If maxValue or minValue is wrong, we will clip the color values.
    const float redShift = value / maxValue - 1.0f;
    const float red = 255.0f * range.limit(1.0f - overlappingScale * redShift * redShift);
    const float greenShift = value / maxValue - 0.5f;
    const float green = 255.0f * range.limit(1.0f - overlappingScale * greenShift * greenShift);
    const float yellowShift = value / maxValue;
    const float blue = 255.0f * range.limit(1.0f - overlappingScale * yellowShift * yellowShift);
    return ColorRGBA(
             static_cast<unsigned char>(red),
             static_cast<unsigned char>(green),
             static_cast<unsigned char>(blue),
             alpha);
  }
};
