/**
 * @file RelativeFieldColorsParameters.h
 *
 * This file declares a representation that contains common thresholds
 * for distinguishing colors of neighboring regions/pixels.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Range.h"
#include <algorithm>

STREAMABLE(RelativeFieldColorsParameters,
{,
  (unsigned char)(15) minWhiteToFieldLuminanceDifference,  /**< White and field color differ at least this much in luminance if they are next to each other in an image. */
  (unsigned char)(15) minWhiteToFieldSaturationDifference, /**< White and field color differ at least this much in saturation if they are next to each other in an image. */
  (unsigned char)(30) minFieldSaturation,  /**< No field pixel is ever less saturated than this. */
  (unsigned char)(100) maxWhiteSaturation, /**< No white pixel is ever more saturated than this. */
  (unsigned char)(200) maxFieldLuminance,  /**< No field pixel is ever lighter than this. */
  (unsigned char)(90) minWhiteLuminance,   /**< No white pixel is ever darker than this. */
  (Rangeuc)(120, 200) fieldHue,            /**< A hue range which is guaranteed to contain the field color. */
});
