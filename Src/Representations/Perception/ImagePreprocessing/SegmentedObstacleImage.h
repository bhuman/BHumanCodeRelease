/**
 * @file SegmentedObstacleImage.h
 *
 * Declares a representation containing a downscaled obstacle segmented version of the camera image.
 *
 * @author Laurens Müller-Groh
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "ImageProcessing/Image.h"
#include "ImageProcessing/PixelTypes.h"
#include "Debugging/DebugImages.h"

STREAMABLE(SegmentedObstacleImage,
{
  void draw() const
  {
    SEND_DEBUG_IMAGE("SegmentedObstacleImage", obstacle);
  },

  (Image<PixelTypes::GrayscaledPixel>) obstacle,
});
