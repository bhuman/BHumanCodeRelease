/**
 * @file Tools/ImageProcessing/Resize.h
 *
 * Functions to resize images.
 *
 * @author Alexis Tsogias, Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/OutStreams.h"

namespace Resize
{
  // shrink color images
  void shrinkNxN(const Image& srcImage, TImage<Image::Pixel>& destImage, unsigned int downScalesExponent);
  void shrink8x8SSE(const Image& srcImage, TImage<Image::Pixel>& destImage);
  void shrink4x4SSE(const Image& srcImage, TImage<Image::Pixel>& destImage);

  // shrink grayscale images
  void shrinkGrayscaleNxN(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage, unsigned int downScalesExponent);
  template<bool avx> void shrinkGrayscale16x16SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkGrayscale8x8SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkGrayscale4x4SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkGrayscale2x2SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);

  // shrink color channels images
  void shrinkColorChannelNxN(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage, unsigned int downScalesExponent);
  template<bool avx> void shrinkColorChannel16x16SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkColorChannel8x8SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkColorChannel4x4SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkColorChannel2x2SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
};
