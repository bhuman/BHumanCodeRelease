/**
 * @file SobelImage.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Debugging/DebugImages.h"

STREAMABLE_WITH_BASE(SobelImage, TImage<PixelTypes::Edge2Pixel>,
{
  mutable TImage<PixelTypes::GrayscaledPixel> debugImage;
  void draw() const
  {
    COMPLEX_IMAGE("SobelImage")
    {
      debugImage.setResolution(this->width, this->height);
      const PixelTypes::Edge2Pixel* pSource = this->operator[](0) - 1;
      const PixelTypes::Edge2Pixel* const end = (*this)[height];
      PixelTypes::GrayscaledPixel* pTarget = debugImage[0] - 1;
      while(++pSource != end)
        *(++pTarget) = static_cast<unsigned char>((std::abs(-128 + pSource->filterX) + std::abs(-128 + pSource->filterY)) << 1);
      SEND_DEBUG_IMAGE("SobelImage", debugImage);
    }
  },
});
