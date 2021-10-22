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

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/ImageProcessing/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Debugging/DebugImages.h"

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
  },

  (unsigned)(0) timestamp,
  (Image<PixelTypes::GrayscaledPixel>) grayscaled,
  (Image<PixelTypes::GrayscaledPixel>) saturated,
  (Image<PixelTypes::HuePixel>) hued,
});
