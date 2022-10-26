/**
 * @file RelativeFieldColors.h
 *
 * This file declares a representation that contains common thresholds
 * for distinguishing colors of neighboring regions/pixels.
 *
 * @author Arne Hasselbring
 * @author Lukas Monnerjahn
 */

#pragma once

#include "Representations/Configuration/RelativeFieldColorsParameters.h"
#include "Streaming/AutoStreamable.h"
#include "Math/Range.h"
#include <algorithm>

STREAMABLE(RelativeFieldColors,
{
  /**
  * Checks whether a color can belong to the field, given the color of a white neighboring reference region/pixel.
  * @param luminance The luminance of the checked color.
  * @param saturation The saturation of the checked color.
  * @param luminanceReference The luminance of a neighboring reference that is known to be white.
  * @param saturationReference The saturation of a neighboring reference that is known to be white.
  * @return Whether the checked color is field.
  */
  bool isFieldNearWhite(unsigned char luminance, unsigned char saturation, unsigned char luminanceReference, unsigned char saturationReference) const
  {
   return luminance <= std::min<int>(luminanceReference - rfcParameters.minWhiteToFieldLuminanceDifference, rfcParameters.maxFieldLuminance) &&
          saturation >= std::max<int>(saturationReference + rfcParameters.minWhiteToFieldSaturationDifference,
                                      std::max(static_cast<unsigned char>(averageSaturation / 2.f), rfcParameters.minFieldSaturation));
  }

  /**
  * Checks whether a color can belong to the field, given the color of a white neighboring reference region/pixel.
  * @param luminance The luminance of the checked color.
  * @param saturation The saturation of the checked color.
  * @param luminanceReference The luminance of a neighboring reference that is known to be white.
  * @param saturationReference The saturation of a neighboring reference that is known to be white.
  * @param hue The hue value of the checked color.
  * @return Whether the checked color is field.
  */
  bool isFieldNearWhite(unsigned char luminance, unsigned char saturation, unsigned char luminanceReference, unsigned char saturationReference, unsigned char hue) const
  {
   return isFieldNearWhite(luminance, saturation, luminanceReference, saturationReference) && rfcParameters.fieldHue.isInside(hue);
  }

  /**
  * Checks whether a color can belong to a white object, given the color of a field-colored neighboring reference region/pixel.
  * @param luminance The luminance of the checked color.
  * @param saturation The saturation of the checked color.
  * @param luminanceReference The luminance of a neighboring reference that is known to be field.
  * @param saturationReference The saturation of a neighboring reference that is known to be field.
  * @return Whether the checked color is white.
  */
  bool isWhiteNearField(unsigned char luminance, unsigned char saturation, unsigned char luminanceReference, unsigned char saturationReference) const
  {
   return luminance >= std::max<int>(luminanceReference + rfcParameters.minWhiteToFieldLuminanceDifference,
                                     std::max(averageLuminance, rfcParameters.minWhiteLuminance)) &&
          saturation <= std::min<int>(saturationReference - rfcParameters.minWhiteToFieldSaturationDifference,
                                      std::min(averageSaturation, rfcParameters.maxWhiteSaturation));
  },

  (RelativeFieldColorsParameters) rfcParameters,   /**< Constant parameters loaded from configuration file. */
  (PixelTypes::GrayscaledPixel) averageLuminance,  /**< Approximated average luminance in the current image. */
  (PixelTypes::GrayscaledPixel) averageSaturation, /**< Approximated average saturation in the current image. */
  (float) averageLuminanceF,                       /**< Approximated average luminance as floating point number */
  (float) averageSaturationF,                      /**< Approximated average saturation as floating point number */
});
