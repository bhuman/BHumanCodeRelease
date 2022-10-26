/**
 * @file Resize.cpp
 *
 * Functions to resize images.
 *
 * @author Felix Thielke
 */

#include "Resize.h"
#include "ImageProcessing/SIMD.h"
#include <array>

void Resize::shrinkY(const unsigned int downScales, const Image<PixelTypes::GrayscaledPixel>& src, PixelTypes::GrayscaledPixel* dest)
{
  const __m128i* pSrc = reinterpret_cast<const __m128i*>(src[0]);

  size_t srcWidth = src.width;
  size_t srcHeight = src.height;

  // Shrink horizontally
  size_t downScalesLeft = downScales;
  for(; downScalesLeft > 2; downScalesLeft -= 3)
  {
    __m128i* pDest = reinterpret_cast<__m128i*>(dest);
    for(size_t n = srcWidth * srcHeight / (8 * 16); n; --n)
    {
      const __m128i shrunk = _mm_packus_epi16(
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
      _mm_store_si128(pDest++, shrunk);
    }
    srcWidth >>= 3;
    pSrc = reinterpret_cast<const __m128i*>(dest);
  }
  if(downScalesLeft > 1)
  {
    __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
    pSrc = reinterpret_cast<const __m128i*>(dest);
    downScalesLeft -= 2;
  }
  if(downScalesLeft)
  {
    __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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

void Resize::shrinkUV(const unsigned int downScales, const Image<PixelTypes::YUYVPixel>& src, unsigned short* dest)
{
  const size_t srcSize = src.width * src.height * 2;
  size_t srcWidth = src.width;
  size_t srcHeight = src.height;
  unsigned int downScalesLeft = downScales;

  // Convert YUV422 to UV and shrink horizontally if needed
  const __m128i* pSrc = reinterpret_cast<const __m128i*>(src[0]);
  __m128i* pDest = reinterpret_cast<__m128i*>(dest);
  static const __m128i shuffleMask = _mm_setr_epi8(0, -1, 1, -1, 4, -1, 5, -1, 8, -1, 9, -1, 12, -1, 13, -1);
  if(downScalesLeft > 2)
  {
    static const __m128i yuyvShuffleMask = _mm_setr_epi8(1, 5, 9, 13, 3, 7, 11, 15, -1, -1, -1, -1, -1, -1, -1, -1);
    for(size_t n = srcSize / (8 * 16); n; --n)
    {
      const __m128i shrunk = _mm_packus_epi16(
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

      _mm_store_si128(pDest++, shrunk);
    }
    downScalesLeft -= 3;
    srcWidth >>= 3;
  }
  else if(downScalesLeft > 1)
  {
    static const __m128i yuyvShuffleMask = _mm_setr_epi8(1, 5, 9, 13, -1, -1, -1, -1, 3, 7, 11, 15, -1, -1, -1, -1);
    for(size_t n = srcSize / (4 * 16); n; --n)
    {
      const __m128i shrunk = _mm_packus_epi16(
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

      _mm_store_si128(pDest++, shrunk);
    }
    downScalesLeft -= 2;
    srcWidth >>= 2;
  }
  else if(downScalesLeft)
  {
    for(size_t n = srcSize / (2 * 16); n; --n)
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
    for(size_t n = srcSize / 16; n; --n)
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
    pSrc = reinterpret_cast<const __m128i*>(dest);
    pDest = reinterpret_cast<__m128i*>(dest);
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
    pSrc = reinterpret_cast<const __m128i*>(dest);
    pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
      pSrc = reinterpret_cast<const __m128i*>(dest);
      __m128i* pDest = reinterpret_cast<__m128i*>(dest);
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
