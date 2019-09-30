/**
 * @file   Sobel.h
 *
 * @author Alexis Tsogias
 */

#pragma once

#include "Image.h"

namespace Sobel
{
  using Image1D = Image<unsigned char>;

  /**
   * Sobel pixels consist of signed chars for the two directions x and y.
   */
  union SobelPixel
  {
    unsigned short index; /**< Used as internal table index */
    struct
    {
      char x, y;
    };
  };

  using SobelImage = Image<SobelPixel>;

  /**
   * Sobel edge detection algorithm optimized with SSE.
   *
   * @param srcImage The source image. It must have a padding of at least one pixel.
   * @param destImage The destination image.
   */
  void sobelSSE(const Image1D& srcImage, SobelImage& destImage);
};
