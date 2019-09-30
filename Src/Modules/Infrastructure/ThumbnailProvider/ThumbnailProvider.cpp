/**
 * @file ThumbnailProvider.cpp
 *
 * Implements a module which calculated a colored or grayscaled thumbnail image.

 * @author Felix Thielke
 */

#include "ThumbnailProvider.h"
#include "Tools/ImageProcessing/SIMD.h"

#include <array>

MAKE_MODULE(ThumbnailProvider, infrastructure)

void ThumbnailProvider::update(Thumbnail& thumbnail)
{
  const unsigned int downScales = useUpperSize && theCameraInfo.camera == CameraInfo::Camera::lower && this->downScales != 0 ? this->downScales - 1 : this->downScales;
  thumbnail.mode = mode;
  thumbnail.scale = 1 << downScales;

  if(downScales == 0)
  {
    if(mode == Thumbnail::grayscaleWithColorClasses)
      combineGrayscaleAndColors(theECImage.grayscaled, theECImage.colored, thumbnail.imageY);
    else
      thumbnail.imageY = theECImage.grayscaled;
  }
  else
  {
    shrinkY(downScales, theECImage.grayscaled, thumbnail.imageY);

    if(mode == Thumbnail::grayscaleWithColorClasses)
    {
      shrinkColors(downScales, theECImage.colored, shrinkedColors);
      combineGrayscaleAndColors(thumbnail.imageY, shrinkedColors, thumbnail.imageY);
    }
  }

  if(mode == Thumbnail::yuv)
    shrinkUV(downScales, theCameraImage, thumbnail.imageUV);
}

void ThumbnailProvider::shrinkY(const unsigned int downScales, const Image<PixelTypes::GrayscaledPixel>& src, Image<PixelTypes::GrayscaledPixel>& dest) const
{
  dest.setResolution(src.width, src.height);
  dest.setResolution(src.width >> downScales, src.height >> downScales);

  const __m128i* pSrc = reinterpret_cast<const __m128i*>(src[0]);

  size_t srcWidth = src.width;
  size_t srcHeight = src.height;

  // Shrink horizontally
  size_t downScalesLeft = downScales;
  for(; downScalesLeft > 2; downScalesLeft -= 3)
  {
    __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = srcWidth * srcHeight / (8 * 16); n; --n)
    {
      const __m128i shrinked = _mm_packus_epi16(
                                 _mm_srli_epi16(
                                   _mm_packs_epi32(
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(_mm_load_si128(pSrc), _mm_setzero_si128()),
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 1), _mm_setzero_si128())
                                     ),
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 2), _mm_setzero_si128()),
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 3), _mm_setzero_si128())
                                     )
                                   ),
                                   3
                                 ),
                                 _mm_srli_epi16(
                                   _mm_packs_epi32(
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 4), _mm_setzero_si128()),
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 5), _mm_setzero_si128())
                                     ),
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 6), _mm_setzero_si128()),
                                       _mm_sad_epu8(_mm_load_si128(pSrc + 7), _mm_setzero_si128())
                                     )
                                   ),
                                   3
                                 )
                               );
      pSrc += 8;
      _mm_store_si128(pDest++, shrinked);
    }
    srcWidth >>= 3;
    pSrc = reinterpret_cast<const __m128i*>(dest[0]);
  }
  if(downScalesLeft > 1)
  {
    __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = srcWidth * srcHeight / (4 * 16); n; --n)
    {
      __m128i p0 = _mm_load_si128(pSrc);
      const __m128i p1 = _mm_load_si128(pSrc + 1);
      __m128i p2 = _mm_load_si128(pSrc + 2);
      const __m128i p3 = _mm_load_si128(pSrc + 3);
      pSrc += 4;

      p0 = _mm_packus_epi16(
             _mm_srli_epi16(_mm_avg_epu8(p0, _mm_slli_epi16(p0, 8)), 8),
             _mm_srli_epi16(_mm_avg_epu8(p1, _mm_slli_epi16(p1, 8)), 8)
           );
      p2 = _mm_packus_epi16(
             _mm_srli_epi16(_mm_avg_epu8(p2, _mm_slli_epi16(p2, 8)), 8),
             _mm_srli_epi16(_mm_avg_epu8(p3, _mm_slli_epi16(p3, 8)), 8)
           );

      _mm_store_si128(pDest++,
                      _mm_packus_epi16(
                        _mm_srli_epi16(_mm_avg_epu8(p0, _mm_slli_epi16(p0, 8)), 8),
                        _mm_srli_epi16(_mm_avg_epu8(p2, _mm_slli_epi16(p2, 8)), 8)
                      )
                     );
    }
    srcWidth >>= 2;
    pSrc = reinterpret_cast<const __m128i*>(dest[0]);
    downScalesLeft -= 2;
  }
  if(downScalesLeft)
  {
    __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = srcWidth * srcHeight / (4 * 16); n; --n)
    {
      const __m128i p0 = _mm_load_si128(pSrc);
      const __m128i p1 = _mm_load_si128(pSrc + 1);
      const __m128i p2 = _mm_load_si128(pSrc + 2);
      const __m128i p3 = _mm_load_si128(pSrc + 3);
      pSrc += 4;

      _mm_store_si128(pDest, _mm_packus_epi16(
                        _mm_srli_epi16(_mm_avg_epu8(p0, _mm_slli_epi16(p0, 8)), 8),
                        _mm_srli_epi16(_mm_avg_epu8(p1, _mm_slli_epi16(p1, 8)), 8)
                      ));
      _mm_store_si128(pDest + 1,
                      _mm_packus_epi16(
                        _mm_srli_epi16(_mm_avg_epu8(p2, _mm_slli_epi16(p2, 8)), 8),
                        _mm_srli_epi16(_mm_avg_epu8(p3, _mm_slli_epi16(p3, 8)), 8)
                      )
                     );
      pDest += 2;
    }
    srcWidth >>= 1;
  }

  // Shrink vertically
  downScalesLeft = downScales;
  if(size_t overshoot = srcWidth % 16) // Row does not fit into SSE registers
  {
    overshoot = 16 - overshoot;
    for(; downScalesLeft > 1; downScalesLeft -= 2)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      srcHeight >>= 2;
      for(size_t nY = srcHeight; nY; --nY)
      {
        for(size_t nX = srcWidth / 16 + 1; nX; --nX)
        {
          const __m128i p0 = _mm_loadu_si128(pSrc);
          const __m128i p1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const char*>(pSrc) + srcWidth));
          const __m128i p2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const char*>(pSrc) + 2 * srcWidth));
          const __m128i p3 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const char*>(pSrc) + 3 * srcWidth));
          pSrc++;

          _mm_storeu_si128(pDest++, _mm_avg_epu8(_mm_avg_epu8(p0, p1), _mm_avg_epu8(p2, p3)));
        }

        pSrc = reinterpret_cast<const __m128i*>(reinterpret_cast<const char*>(pSrc) + 3 * srcWidth - overshoot);
        pDest = reinterpret_cast<__m128i*>(reinterpret_cast<char*>(pDest) - overshoot);
      }
    }
    if(downScalesLeft)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      for(size_t nY = srcHeight / 2; nY; --nY)
      {
        for(size_t nX = srcWidth / 16 + 1; nX; --nX)
        {
          const __m128i p0 = _mm_loadu_si128(pSrc);
          const __m128i p1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const char*>(pSrc) + srcWidth));
          pSrc++;

          _mm_storeu_si128(pDest++, _mm_avg_epu8(p0, p1));
        }

        pSrc = reinterpret_cast<const __m128i*>(reinterpret_cast<const char*>(pSrc) + srcWidth - overshoot);
        pDest = reinterpret_cast<__m128i*>(reinterpret_cast<char*>(pDest) - overshoot);
      }
    }
  }
  else  // Row fits into SSE registers
  {
    for(; downScalesLeft > 1; downScalesLeft -= 2)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      srcHeight >>= 2;
      for(size_t nY = srcHeight; nY; --nY)
      {
        for(size_t nX = srcWidth / 16; nX; --nX)
        {
          const __m128i p0 = _mm_load_si128(pSrc);
          const __m128i p1 = _mm_load_si128(pSrc + srcWidth / 16);
          const __m128i p2 = _mm_load_si128(pSrc + 2 * srcWidth / 16);
          const __m128i p3 = _mm_load_si128(pSrc + 3 * srcWidth / 16);
          pSrc++;

          _mm_store_si128(pDest++, _mm_avg_epu8(_mm_avg_epu8(p0, p1), _mm_avg_epu8(p2, p3)));
        }

        pSrc += 3 * srcWidth / 16;
      }
    }
    if(downScalesLeft)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      for(size_t nY = srcHeight / 2; nY; --nY)
      {
        for(size_t nX = srcWidth / 16; nX; --nX)
        {
          const __m128i p0 = _mm_load_si128(pSrc);
          const __m128i p1 = _mm_load_si128(pSrc + srcWidth / 16);
          pSrc++;

          _mm_stream_si128(pDest++, _mm_avg_epu8(p0, p1));
        }

        pSrc += srcWidth / 16;
      }
    }
  }
}

void ThumbnailProvider::shrinkColors(const unsigned int downScales, const Image<PixelTypes::ColoredPixel>& src, Image<PixelTypes::ColoredPixel>& dest) const
{
  dest.setResolution(src.width >> downScales, src.height >> downScales);

  // TODO: optimize this
  for(unsigned int yOut = 0; yOut < dest.height; yOut++)
  {
    for(unsigned int xOut = 0; xOut < dest.width; xOut++)
    {
      std::array<unsigned int, PixelTypes::ColoredPixel::numOfColors> votes;
      votes.fill(0);

      for(unsigned int yIn = yOut << downScales; yIn < (yOut + 1) << downScales; yIn++)
        for(unsigned int xIn = xOut << downScales; xIn < (xOut + 1) << downScales; xIn++)
          votes[src(xIn, yIn)]++;

      PixelTypes::ColoredPixel maxElem;
      unsigned int max = 0;
      for(size_t i = 0; i < votes.size(); i++)
      {
        if(votes[i] > max)
        {
          max = votes[i];
          maxElem = static_cast<PixelTypes::ColoredPixel>(i);
        }
      }

      dest(xOut, yOut) = maxElem;
    }
  }
}

void ThumbnailProvider::shrinkUV(const unsigned int downScales, const CameraImage& src, Image<unsigned short>& dest) const
{
  dest.setResolution(src.width, src.height);
  dest.setResolution(src.width >> downScales, src.height >> (downScales + 1));

  size_t srcWidth = src.width;
  size_t srcHeight = src.height;
  unsigned int downScalesLeft = downScales;

  // Convert YUV422 to UV and shrink horizontally if needed
  const __m128i* pSrc = reinterpret_cast<const __m128i*>(src[0]);
  __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
  static const __m128i shuffleMask = _mm_setr_epi8(0, -1, 1, -1, 4, -1, 5, -1, 8, -1, 9, -1, 12, -1, 13, -1);
  if(downScalesLeft > 2)
  {
    static const __m128i yuyvShuffleMask = _mm_setr_epi8(1, 5, 9, 13, 3, 7, 11, 15, -1, -1, -1, -1, -1, -1, -1, -1);
    for(size_t n = theCameraInfo.height * theCameraInfo.width / (8 * 16); n; --n)
    {
      const __m128i shrinked = _mm_packus_epi16(
                                 _mm_srli_epi16(
                                   _mm_packs_epi32(
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 1), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 2), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 3), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       )
                                     ),
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 4), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 5), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 6), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 7), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       )
                                     )
                                   ),
                                   3
                                 ),
                                 _mm_srli_epi16(
                                   _mm_packs_epi32(
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 8), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 9), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 10), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 11), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       )
                                     ),
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 12), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 13), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_unpacklo_epi32(
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 14), yuyvShuffleMask),
                                           _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 15), yuyvShuffleMask)
                                         ),
                                         _mm_setzero_si128()
                                       )
                                     )
                                   ),
                                   3
                                 )
                               );
      pSrc += 16;

      _mm_store_si128(pDest++, shrinked);
    }
    downScalesLeft -= 3;
    srcWidth >>= 3;
  }
  else if(downScalesLeft > 1)
  {
    static const __m128i yuyvShuffleMask = _mm_setr_epi8(1, 5, 9, 13, -1, -1, -1, -1, 3, 7, 11, 15, -1, -1, -1, -1);
    for(size_t n = theCameraInfo.height * theCameraInfo.width / (4 * 16); n; --n)
    {
      const __m128i shrinked = _mm_packus_epi16(
                                 _mm_srli_epi16(
                                   _mm_packs_epi32(
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 1), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       )
                                     ),
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 2), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 3), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       )
                                     )
                                   ),
                                   2
                                 ),
                                 _mm_srli_epi16(
                                   _mm_packs_epi32(
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 4), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 5), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       )
                                     ),
                                     _mm_packs_epi32(
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 6), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       ),
                                       _mm_sad_epu8(
                                         _mm_shuffle_epi8(_mm_loadu_si128(pSrc + 7), yuyvShuffleMask),
                                         _mm_setzero_si128()
                                       )
                                     )
                                   ),
                                   2
                                 )
                               );
      pSrc += 8;

      _mm_store_si128(pDest++, shrinked);
    }
    downScalesLeft -= 2;
    srcWidth >>= 2;
  }
  else if(downScalesLeft)
  {
    for(size_t n = theCameraInfo.height * theCameraInfo.width / (2 * 16); n; --n)
    {
      const __m128i left = _mm_packus_epi16(
                             _mm_srli_epi16(_mm_loadu_si128(pSrc), 8),
                             _mm_srli_epi16(_mm_loadu_si128(pSrc + 1), 8)
                           );
      const __m128i right = _mm_packus_epi16(
                              _mm_srli_epi16(_mm_loadu_si128(pSrc + 2), 8),
                              _mm_srli_epi16(_mm_loadu_si128(pSrc + 3), 8)
                            );
      pSrc += 4;

      _mm_store_si128(pDest++,
                      _mm_packus_epi16(
                        _mm_shuffle_epi8(_mm_avg_epu8(left, _mm_srli_epi32(left, 16)), shuffleMask),
                        _mm_shuffle_epi8(_mm_avg_epu8(right, _mm_srli_epi32(right, 16)), shuffleMask)
                      )
                     );
    }
    downScalesLeft--;
    srcWidth >>= 1;
  }
  else
  {
    for(size_t n = theCameraInfo.height * theCameraInfo.width / 16; n; --n)
    {
      const __m128i p0 = _mm_loadu_si128(pSrc);
      const __m128i p1 = _mm_loadu_si128(pSrc + 1);
      pSrc += 2;

      _mm_store_si128(pDest++,
                      _mm_packus_epi16(
                        _mm_srli_epi16(p0, 8),
                        _mm_srli_epi16(p1, 8)
                      )
                     );
    }
  }

  // Shrink horizontally
  for(; downScalesLeft > 1; downScalesLeft -= 2)
  {
    pSrc = reinterpret_cast<const __m128i*>(dest[0]);
    pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = srcWidth * srcHeight / (2 * 16); n; --n)
    {
      __m128i p0 = _mm_load_si128(pSrc);
      const __m128i p1 = _mm_load_si128(pSrc + 1);
      __m128i p2 = _mm_load_si128(pSrc + 2);
      const __m128i p3 = _mm_load_si128(pSrc + 3);
      pSrc += 4;

      p0 = _mm_packus_epi16(
             _mm_shuffle_epi8(_mm_avg_epu8(p0, _mm_srli_epi32(p0, 16)), shuffleMask),
             _mm_shuffle_epi8(_mm_avg_epu8(p1, _mm_srli_epi32(p1, 16)), shuffleMask)
           );
      p2 = _mm_packus_epi16(
             _mm_shuffle_epi8(_mm_avg_epu8(p2, _mm_srli_epi32(p2, 16)), shuffleMask),
             _mm_shuffle_epi8(_mm_avg_epu8(p3, _mm_srli_epi32(p3, 16)), shuffleMask)
           );

      _mm_store_si128(pDest++,
                      _mm_packus_epi16(
                        _mm_shuffle_epi8(_mm_avg_epu8(p0, _mm_srli_epi32(p0, 16)), shuffleMask),
                        _mm_shuffle_epi8(_mm_avg_epu8(p2, _mm_srli_epi32(p2, 16)), shuffleMask)
                      )
                     );
    }
    srcWidth >>= 2;
  }
  if(downScalesLeft)
  {
    pSrc = reinterpret_cast<const __m128i*>(dest[0]);
    pDest = reinterpret_cast<__m128i*>(dest[0]);
    for(size_t n = srcWidth * srcHeight / (2 * 16); n; --n)
    {
      const __m128i p0 = _mm_load_si128(pSrc);
      const __m128i p1 = _mm_load_si128(pSrc + 1);
      const __m128i p2 = _mm_load_si128(pSrc + 2);
      const __m128i p3 = _mm_load_si128(pSrc + 3);
      pSrc += 4;

      _mm_store_si128(pDest,
                      _mm_packus_epi16(
                        _mm_shuffle_epi8(_mm_avg_epu8(p0, _mm_srli_epi32(p0, 16)), shuffleMask),
                        _mm_shuffle_epi8(_mm_avg_epu8(p1, _mm_srli_epi32(p1, 16)), shuffleMask)
                      )
                     );
      _mm_store_si128(pDest + 1,
                      _mm_packus_epi16(
                        _mm_shuffle_epi8(_mm_avg_epu8(p2, _mm_srli_epi32(p2, 16)), shuffleMask),
                        _mm_shuffle_epi8(_mm_avg_epu8(p3, _mm_srli_epi32(p3, 16)), shuffleMask)
                      )
                     );
      pDest += 2;
    }
    srcWidth >>= 1;
  }

  // Shrink vertically
  downScalesLeft = downScales + 1;
  if(size_t overshoot = srcWidth % 8) // Row does not fit into SSE registers
  {
    overshoot = 8 - overshoot;
    for(; downScalesLeft > 1; downScalesLeft -= 2)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      srcHeight >>= 2;
      for(size_t nY = srcHeight; nY; --nY)
      {
        for(size_t nX = srcWidth / 8 + 1; nX; --nX)
        {
          const __m128i p0 = _mm_loadu_si128(pSrc);
          const __m128i p1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + srcWidth));
          const __m128i p2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + 2 * srcWidth));
          const __m128i p3 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + 3 * srcWidth));
          pSrc++;

          _mm_storeu_si128(pDest++, _mm_avg_epu8(_mm_avg_epu8(p0, p1), _mm_avg_epu8(p2, p3)));
        }

        pSrc = reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + 3 * srcWidth - overshoot);
        pDest = reinterpret_cast<__m128i*>(reinterpret_cast<short*>(pDest) - overshoot);
      }
    }
    if(downScalesLeft)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      for(size_t nY = srcHeight / 2; nY; --nY)
      {
        for(size_t nX = srcWidth / 8 + 1; nX; --nX)
        {
          const __m128i p0 = _mm_loadu_si128(pSrc);
          const __m128i p1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + srcWidth));
          pSrc++;

          _mm_storeu_si128(pDest++, _mm_avg_epu8(p0, p1));
        }

        pSrc = reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + srcWidth - overshoot);
        pDest = reinterpret_cast<__m128i*>(reinterpret_cast<short*>(pDest) - overshoot);
      }
    }
  }
  else // Row fits into SSE registers
  {
    for(; downScalesLeft > 1; downScalesLeft -= 2)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      srcHeight >>= 2;
      for(size_t nY = srcHeight; nY; --nY)
      {
        for(size_t nX = srcWidth / 8; nX; --nX)
        {
          const __m128i p0 = _mm_load_si128(pSrc);
          const __m128i p1 = _mm_load_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + srcWidth));
          const __m128i p2 = _mm_load_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + 2 * srcWidth));
          const __m128i p3 = _mm_load_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + 3 * srcWidth));
          pSrc++;

          _mm_store_si128(pDest++, _mm_avg_epu8(_mm_avg_epu8(p0, p1), _mm_avg_epu8(p2, p3)));
        }

        pSrc += 3 * srcWidth / 8;
      }
    }
    if(downScalesLeft)
    {
      pSrc = reinterpret_cast<const __m128i*>(dest[0]);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
      for(size_t nY = srcHeight / 2; nY; --nY)
      {
        for(size_t nX = srcWidth / 8; nX; --nX)
        {
          const __m128i p0 = _mm_load_si128(pSrc);
          const __m128i p1 = _mm_load_si128(reinterpret_cast<const __m128i*>(reinterpret_cast<const short*>(pSrc) + srcWidth));
          pSrc++;

          _mm_store_si128(pDest++, _mm_avg_epu8(p0, p1));
        }

        pSrc += srcWidth / 8;
      }
    }
  }
}

void ThumbnailProvider::combineGrayscaleAndColors(const Image<PixelTypes::GrayscaledPixel>& y, const Image<PixelTypes::ColoredPixel>& c, Image<PixelTypes::GrayscaledPixel>& dest) const
{
  const __m128i* pY = reinterpret_cast<const __m128i*>(y[0]);
  const __m128i* pC = reinterpret_cast<const __m128i*>(c[0]);
  __m128i* pDest = reinterpret_cast<__m128i*>(dest[0]);
  for(size_t n = y.width * y.width / 16; n; --n)
  {
    _mm_store_si128(
      pDest++,
      _mm_or_si128(
        _mm_and_si128(_mm_load_si128(pY++), _mm_set1_epi8(char(0xFC))),
        _mm_load_si128(pC++)
      )
    );
  }
}
