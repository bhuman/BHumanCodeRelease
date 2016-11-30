/**
 * @file Tools/ImageProcessing/Resize.h
 *
 * Functions to resize images.
 *
 * @author Alexis Tsogias, Felix Thielke
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

  // shrink color images and convert them to grayscale on the fly
  void shrinkAndConvertToGrayscaleNxN(const Image& srcImage, TImage<unsigned char>& destImage, unsigned int downScalesExponent);
  void shrinkAndConvertToGrayscale8x8SSE(const Image& srcImage, TImage<unsigned char>& destImage);
  void shrinkAndConvertToGrayscale4x4SSE(const Image& srcImage, TImage<unsigned char>& destImage);

  // shrink grayscale images
  void shrinkGrayscaleNxN(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage, unsigned int downScalesExponent);
  template<bool avx> void shrinkGrayscale16x16SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkGrayscale8x8SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkGrayscale4x4SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
  template<bool avx> void shrinkGrayscale2x2SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage);
};

