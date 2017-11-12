/**
 * @file ECImage.h
 *
 * Declares a represention containing both a color classified and a grayscale
 * version of the camera image.
 * It is advised to use this representation for all further image processing.
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Debugging/DebugImages.h"

/**
 * A represention containing both a color classified and a grayscale version of
 * the camera image.
 * It is advised to use this representation for all further image processing.
 */
STREAMABLE(ECImage,
{
  void draw() const
  {
    SEND_DEBUG_IMAGE("ColoredImage", colored);
    SEND_DEBUG_IMAGE("GrayscaledImage", grayscaled);
    SEND_DEBUG_IMAGE("Ued", ued);
    SEND_DEBUG_IMAGE("Ved", ved);
    SEND_DEBUG_IMAGE("SaturatedImage", saturated);
    SEND_DEBUG_IMAGE("HuedImage", hued);
  },

  (unsigned)(0) timeStamp,
  (TImage<PixelTypes::GrayscaledPixel>) grayscaled,
  (TImage<PixelTypes::GrayscaledPixel>) ued,
  (TImage<PixelTypes::GrayscaledPixel>) ved,
  (TImage<PixelTypes::ColoredPixel>) colored,
  (TImage<PixelTypes::GrayscaledPixel>) saturated,
  (TImage<PixelTypes::HuePixel>) hued,
});

STREAMABLE(ThumbnailCImage,
{
  void draw() const
  {
    SEND_DEBUG_IMAGE("ThumbnailColoredImage", colored);
    SEND_DEBUG_IMAGE("ThumbnailSaturatedImage", saturated);
    SEND_DEBUG_IMAGE("ThumbnailHuedImage", hued);
  },

  (TImage<PixelTypes::ColoredPixel>) colored,
  (TImage<PixelTypes::GrayscaledPixel>) saturated,
  (TImage<PixelTypes::HuePixel>) hued,
});
