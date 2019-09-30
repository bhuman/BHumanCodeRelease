/**
 * The file declares a contrast normalized Sobel (CNS) image, i.e. an image of
 * CNS responses.
 *
 * @author Udo Frese
 * @author Thomas Röfer
 */

#pragma once

#include <cmath>
#include <cstdlib>
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ImageProcessing/Image.h"

/**
 * The response of the contrast normalized Sobel (CNS) filter at a single point.
 * A CNS response is a vector with length <=1 that points into the direction
 * of the gradient. The length defines, which fraction of the weighted image
 * contrast at the respective point comes from the linear gradient. Hence,
 * the length is an illumination independent indicator of quality.

 * \f{equation}{
 *   CNS (x,y) = \frac{
 *     \frac{\sqrt{2}}{16} \left(
 *       \begin{pmatrix} -1 & 0 & 1 \\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{pmatrix} * I,
 *       \begin{pmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{pmatrix} * I
 *     \right)^T
 *   }{\sqrt{
 *     \frac{1}{16} \begin{pmatrix} 1 & 2 & 1 \\ 2 & 4 & 2 \\ 1 & 2 & 1 \end{pmatrix} * I^2
 *     - \left(
 *       \frac{1}{16} \begin{pmatrix} 1 & 2 & 1 \\ 2 & 4 & 2 \\ 1 & 2 & 1 \end{pmatrix} * I
 *     \right)^2
 *     + \sigma^2
 *   }} (x, y)
 * \f}
 *
 * Where $I$ is the image, \f$\ast\f$ denotes convolution and all other operations are
 * applied pointwise. The nominator is a simple Sobel image gradient, while the
 * denominator normalizes with respect to contrast. It is the Gaussian weighted image
 * std. deviation with a regularizer parameter \f$\sigma\f$. This regularizer avoids avoids
 * $0/0$ situation for a constant image.
 *
 * Note, that the Gaussian used in the contrast normalization is compatible with the
 * Sobel filters, since both Sobel are equal to the Gaussian times a linear Gradient. So
 * one can think of the nominator as a Gaussian weighted gradient and the denominator as
 * a Gaussian weighted image contrast.
 *
 * The memory layout of this class, i.e. having two consecutive signed
 * bytes is hard-coded in the respective routines. Don't change.
 */
struct CNSResponse : public PixelTypes::Edge2Pixel
{
  /**
   * Factor by which the float responses are scaled to get integer.
   * This factor is hard coded in the SSE routines. Do not change.
   */
  static constexpr int SCALE = 1 << 7;

  /** Offset added to the scaled float responses to make them unsigned. */
  static constexpr int OFFSET = 128;

  /**
   * Both components are between -1<..<+1 scaled by 2^7 and shifted by 128 to unsigned char.
   * The vector length of the entry must be <2^7.
   * \f{equation}{
   *   filterX = CNS(x,y)_x SCALE \quad
   *   filterY = CNS(x,y)_y SCALE
   * \f}
   */
  //filterX inside base class
  //filterY inside base class

  /** Checks, whether both numbers are in a valid range. */
  bool valid() const
  {
    int x = filterX - OFFSET;
    int y = filterY - OFFSET;
    return x * x + y * y < (SCALE + 1) * (SCALE + 1); // allow for 1 rounding error
  }

  /** Whether the pixel denotes a 0-vector. */
  bool isZero() const
  {
    return filterX == OFFSET && filterY == OFFSET;
  }

  /** Uninitialized constructor. */
  CNSResponse() : PixelTypes::Edge2Pixel(OFFSET, OFFSET) {};

  /** Initializes CNSResponse incl. clipping and scaling. */
  CNSResponse(float filterX, float filterY)
  {
    filterX = static_cast<float>(SCALE) * filterX + static_cast<float>(OFFSET);
    filterY = static_cast<float>(SCALE) * filterY + static_cast<float>(OFFSET);
    if(filterX < 0)
      filterX = 0;
    else if(filterX > 0xff)
      filterX = 0xff;
    this->filterX = static_cast<unsigned char>(filterX);
    if(filterY < 0)
      filterY = 0;
    else if(filterY > 0xff)
      filterY = 0xff;
    this->filterY = static_cast<unsigned char>(filterY);
  }

  /** Returns the maximal difference in either x or y. */
  int diff(const CNSResponse& other) const
  {
    return std::max(std::abs(filterX - other.filterX),
                    std::abs(filterY - other.filterY));
  }

  /** Returns x as float (-1 .. +1). */
  float xFloat() const
  {
    return (static_cast<float>(filterX) - static_cast<float>(OFFSET)) / static_cast<float>(SCALE);
  }

  /** Returns y as float (-1 .. +1). */
  float yFloat() const
  {
    return (static_cast<float>(filterY) - static_cast<float>(OFFSET)) / static_cast<float>(SCALE);
  }

  /** Return the norm as float [0 .. 1]. */
  float normFloat() const
  {
    float dX = xFloat();
    float dY = yFloat();
    return std::sqrt(dX * dX + dY * dY);
  }
};

STREAMABLE_WITH_BASE(CNSImage, Image<CNSResponse>,
{
  CNSImage() : Image<CNSResponse>(640, 480, 640 * 64 * sizeof(CNSResponse))
  {
    std::memset(reinterpret_cast<char*>((*this)[-64]), CNSResponse::OFFSET, width * (128 + height) * sizeof(CNSResponse));
  }

  void draw() const
  {
    SEND_DEBUG_IMAGE("CNSImage", *this, PixelTypes::Edge2);
  },
});
