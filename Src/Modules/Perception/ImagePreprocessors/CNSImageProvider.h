/**
 * This file declares a module that calculates contrast normalized Sobel (cns) images.
 * @author Udo Frese
 * @author Thomas Röfer
 * @Author Lukas Post
 */

#pragma once

#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Tools/Module/Module.h"

MODULE(CNSImageProvider,
{,
  REQUIRES(CNSRegions),
  REQUIRES(ECImage),
  PROVIDES_WITHOUT_MODIFY(CNSImage),
  LOADS_PARAMETERS(
  {,
    (float) minContrast, /**< Gradiants below this threshold are ignored in a gradual way. */
    (bool) fullImage, /**< Always compute complete CNS image. */
  }),
});

class CNSImageProvider : public CNSImageProviderBase
{
  /**
   * Computes the cns image from the grayscale image.
   * If \c doBlur is true, the source image is blurred by a 3*3 Gaussian before computing
   * the cns image.
   */
  void update(CNSImage& cnsImage) override;

  /**
   * Computes the cns response image in an SSE2 implementation
   * The image must be passed in \c src, where pixel \c src(x,y) corresponds to
   * \c src[x + y * width].
   * The result is stored in \c dst, where pixel \c dst(x,y) corresponds to
   * \c dst[x + y * width]. \c cns(x,y) is the result of the CNS computations based on
   * a 3*3 filter centered at \c src(x,y).
   */
  static void cnsResponse(const unsigned char* src, int width, int height,
                          int srcOfs, short* cns, float regVar);
};
