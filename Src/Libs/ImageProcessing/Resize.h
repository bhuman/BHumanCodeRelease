/**
 * @file Resize.h
 *
 * Functions to resize images.
 *
 * @author Felix Thielke
 */

#pragma once

#include "ImageProcessing/Image.h"
#include "ImageProcessing/PixelTypes.h"

namespace Resize
{
  void shrinkY(const unsigned int downScales, const Image<PixelTypes::GrayscaledPixel>& src, PixelTypes::GrayscaledPixel* dest);

  inline void shrinkY(const unsigned int downScales, const Image<PixelTypes::GrayscaledPixel>& src, Image<PixelTypes::GrayscaledPixel>& dest)
  {
    dest.setResolution(src.width, src.height); // The shrinking algorithm uses more than the final downscaled size.
    dest.setResolution(src.width >> downScales, src.height >> downScales);
    shrinkY(downScales, src, dest[0]);
  }

  void shrinkUV(const unsigned int downScales, const Image<PixelTypes::YUYVPixel>& src, unsigned short* dest);

  inline void shrinkUV(const unsigned int downScales, const Image<PixelTypes::YUYVPixel>& src, Image<unsigned short>& dest)
  {
    dest.setResolution(src.width, src.height); // The shrinking algorithm uses more than the final downscaled size.
    dest.setResolution(src.width >> downScales, src.height >> (downScales + 1));
    shrinkUV(downScales, src, dest[0]);
  }
}
