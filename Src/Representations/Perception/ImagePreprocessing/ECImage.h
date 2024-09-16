/**
 * @file ECImage.h
 *
 * Declares a representation containing both a color classified and a grayscale
 * version of the camera image.
 * It is advised to use this representation for all further image processing.
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "ImageProcessing/Image.h"
#include "ImageProcessing/PixelTypes.h"
#include "Debugging/DebugImages.h"

/**
 * A representation containing both a color classified and a grayscale version of
 * the camera image.
 * It is advised to use this representation for all further image processing.
 */
STREAMABLE(ECImage,
{
  void draw() const
  {
    SEND_DEBUG_IMAGE("GrayscaledImage", grayscaled);
    SEND_DEBUG_IMAGE("SaturatedImage", saturated);
    SEND_DEBUG_IMAGE("HuedImage", hued);
    SEND_DEBUG_IMAGE("BlueChromaticityImage", blueChromaticity);
    SEND_DEBUG_IMAGE("RedChromaticityImage", redChromaticity);
  },

  (unsigned)(0) timestamp,
  (Image<PixelTypes::GrayscaledPixel>) grayscaled,
  (Image<PixelTypes::GrayscaledPixel>) saturated,
  (Image<PixelTypes::HuePixel>) hued,
  (Image<PixelTypes::GrayscaledPixel>) blueChromaticity,
  (Image<PixelTypes::GrayscaledPixel>) redChromaticity,
});

/**
 * A wrapper representation for the ECImage that allows not to store one.
 * This allows to stream an ECImage only if it is necessary.
 */
STREAMABLE(OptionalECImage,
{,
  (std::optional<ECImage>) image,
});
