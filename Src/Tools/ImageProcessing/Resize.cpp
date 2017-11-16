/**
 * @file Tools/ImageProcessing/Resize.cpp
 *
 * Functions to resize images.
 *
 * @author Alexis Tsogias, Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Resize.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/AVX.h"

#include <cstring>
#include <cstddef>
#include <cmath>

namespace Resize
{
  void shrinkNxN(const Image& srcImage, TImage<Image::Pixel>& destImage, unsigned int downScalesExponent)
  {
    const int scaleFactor = static_cast<int>(1 << downScalesExponent);
    const int averagedPixels = scaleFactor * scaleFactor;
    const int width = srcImage.width;
    const int height = srcImage.height;

    ASSERT(width % scaleFactor == 0);
    ASSERT(height % scaleFactor == 0);

    destImage.setResolution(width / scaleFactor, height / scaleFactor);

    struct pixelSum
    {
      int y;
      int cb;
      int cr;
    };
    pixelSum* summs = new pixelSum[destImage.width];
    memset(summs, 0, destImage.width * sizeof(pixelSum));

    const Image::Pixel* pSrc;
    TImage<Image::Pixel>::PixelType* pDest = nullptr;
    pixelSum* pSumms;

    for(int y = 0; y < height; ++y)
    {
      if(y % scaleFactor == 0)
      {
        pDest = destImage[y / scaleFactor];
      }
      pSrc = srcImage[y];
      pSumms = summs;
      for(int x = 0; x < width; x += scaleFactor, ++pSumms)
      {
        for(int i = 0; i < scaleFactor; ++i, ++pSrc)
        {
          pSumms->y += pSrc->y;
          pSumms->cb += pSrc->cb;
          pSumms->cr += pSrc->cr;
        }
      }

      if(y % scaleFactor == scaleFactor - 1)
      {
        pSumms = summs;
        for(int i = 0; i < destImage.width; ++i, ++pSumms, ++pDest)
        {
          pDest->y = static_cast<unsigned char>(pSumms->y / averagedPixels);
          pDest->cb = static_cast<unsigned char>(pSumms->cb / averagedPixels);
          pDest->cr = static_cast<unsigned char>(pSumms->cr / averagedPixels);
        }
        memset(summs, 0, destImage.width * sizeof(pixelSum));
      }
    }

    delete[] summs;
  }

  void shrink8x8SSE(const Image& srcImage, TImage<Image::Pixel>& destImage)
  {
    const int scaleFactor = 8;
    const int averagedPixels = scaleFactor * scaleFactor;
    const int width = srcImage.width;
    const int height = srcImage.height;

    ASSERT(width % scaleFactor == 0);
    ASSERT(height % scaleFactor == 0);

    destImage.setResolution(width / scaleFactor, height / scaleFactor);

    const __m128i zero = _mm_setzero_si128();
    const int summsSize = destImage.width * 16;
    __m128i* summs = reinterpret_cast<__m128i*>(Memory::alignedMalloc(summsSize, 16));
    memset(summs, 0, summsSize);

    const Image::Pixel* pSrc;
    TImage<Image::Pixel>::PixelType* pDest = nullptr;
    __m128i* pSumms;

    __m128i tmp;
    __m128i lower;
    __m128i upper;

    for(int y = 0; y < height; ++y)
    {
      if(y % scaleFactor == 0)
      {
        pDest = destImage[y / scaleFactor];
      }
      pSrc = srcImage[y];
      pSumms = summs;
      for(int x = 0; x < width; x += scaleFactor, pSrc += scaleFactor, ++pSumms)
      {
        tmp = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc));
        lower = _mm_unpacklo_epi8(tmp, zero);
        upper = _mm_unpackhi_epi8(tmp, zero);
        *pSumms = _mm_add_epi16(*pSumms, lower);
        *pSumms = _mm_add_epi16(*pSumms, upper);

        tmp = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc + scaleFactor / 2));
        lower = _mm_unpacklo_epi8(tmp, zero);
        upper = _mm_unpackhi_epi8(tmp, zero);
        *pSumms = _mm_add_epi16(*pSumms, lower);
        *pSumms = _mm_add_epi16(*pSumms, upper);
      }

      if(y % scaleFactor == scaleFactor - 1)
      {
        pSumms = summs;
        for(int i = 0; i < destImage.width; ++i, ++pSumms, ++pDest)
        {
          const short* ptr = reinterpret_cast<short*>(pSumms);

          const short sumY = ptr[offsetof(Image::Pixel, y)] + ptr[offsetof(Image::Pixel, y) + sizeof(Image::Pixel)];
          const short sumCb = ptr[offsetof(Image::Pixel, cb)] + ptr[offsetof(Image::Pixel, cb) + sizeof(Image::Pixel)];
          const short sumCr = ptr[offsetof(Image::Pixel, cr)] + ptr[offsetof(Image::Pixel, cr) + sizeof(Image::Pixel)];

          pDest->y = static_cast<char>(sumY / averagedPixels);
          pDest->cb = static_cast<char>(sumCb / averagedPixels);
          pDest->cr = static_cast<char>(sumCr / averagedPixels);
        }
        memset(summs, 0, summsSize);
      }
    }
    Memory::alignedFree(summs);
  }

  void shrink4x4SSE(const Image& srcImage, TImage<Image::Pixel>& destImage)
  {
    const int scaleFactor = 4;
    const int averagedPixels = scaleFactor * scaleFactor;
    const int width = srcImage.width;
    const int height = srcImage.height;

    ASSERT(width % scaleFactor == 0);
    ASSERT(height % scaleFactor == 0);

    destImage.setResolution(width / scaleFactor, height / scaleFactor);

    const __m128i zero = _mm_setzero_si128();
    const int summsSize = destImage.width * 16;
    __m128i* summs = reinterpret_cast<__m128i*>(Memory::alignedMalloc(summsSize, 16));
    memset(summs, 0, summsSize);

    const Image::Pixel* pSrc;
    TImage<Image::Pixel>::PixelType* pDest = nullptr;
    __m128i* pSumms;

    __m128i tmp;
    __m128i lower;
    __m128i upper;

    for(int y = 0; y < height; ++y)
    {
      if(y % scaleFactor == 0)
      {
        pDest = destImage[y / scaleFactor];
      }
      pSrc = srcImage[y];
      pSumms = summs;
      for(int x = 0; x < width; x += scaleFactor, pSrc += scaleFactor, ++pSumms)
      {
        tmp = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc));
        lower = _mm_unpacklo_epi8(tmp, zero);
        upper = _mm_unpackhi_epi8(tmp, zero);
        *pSumms = _mm_add_epi16(*pSumms, lower);
        *pSumms = _mm_add_epi16(*pSumms, upper);
      }

      if(y % scaleFactor == scaleFactor - 1)
      {
        pSumms = summs;
        for(int i = 0; i < destImage.width; ++i, ++pSumms, ++pDest)
        {
          const short* ptr = reinterpret_cast<short*>(pSumms);

          const short sumY = ptr[offsetof(Image::Pixel, y)] + ptr[offsetof(Image::Pixel, y) + sizeof(Image::Pixel)];
          const short sumCb = ptr[offsetof(Image::Pixel, cb)] + ptr[offsetof(Image::Pixel, cb) + sizeof(Image::Pixel)];
          const short sumCr = ptr[offsetof(Image::Pixel, cr)] + ptr[offsetof(Image::Pixel, cr) + sizeof(Image::Pixel)];

          pDest->y = static_cast<char>(sumY / averagedPixels);
          pDest->cb = static_cast<char>(sumCb / averagedPixels);
          pDest->cr = static_cast<char>(sumCr / averagedPixels);
        }
        memset(summs, 0, summsSize);
      }
    }
    Memory::alignedFree(summs);
  }

  void shrinkGrayscaleNxN(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage, unsigned int downScalesExponent)
  {
    const int scaleFactor = static_cast<int>(1 << downScalesExponent);
    const int width = srcImage.width;
    const int height = srcImage.height;

    ASSERT(scaleFactor > 1);
    ASSERT(width % scaleFactor == 0);
    ASSERT(height % scaleFactor == 0);

    if(width % 16 == 0)
    {
      if(_supportsAVX2 && width % 32 == 0)
      {
        switch(downScalesExponent)
        {
          case 1:
            shrinkGrayscale2x2SSE<true>(srcImage, destImage);
            return;
          case 2:
            shrinkGrayscale4x4SSE<true>(srcImage, destImage);
            return;
          case 3:
            shrinkGrayscale8x8SSE<true>(srcImage, destImage);
            return;
          case 4:
            shrinkGrayscale16x16SSE<true>(srcImage, destImage);
            return;
        }
      }
      else
      {
        switch(downScalesExponent)
        {
          case 1:
            shrinkGrayscale2x2SSE<false>(srcImage, destImage);
            return;
          case 2:
            shrinkGrayscale4x4SSE<false>(srcImage, destImage);
            return;
          case 3:
            shrinkGrayscale8x8SSE<false>(srcImage, destImage);
            return;
          case 4:
            shrinkGrayscale16x16SSE<false>(srcImage, destImage);
            return;
        }
      }
    }

    const int averagedPixels = scaleFactor * scaleFactor;

    destImage.setResolution(width >> downScalesExponent, height >> downScalesExponent);

    unsigned int* summs = new unsigned int[destImage.width];
    memset(summs, 0, destImage.width * sizeof(unsigned int));

    const unsigned char* pSrc;
    TImage<unsigned char>::PixelType* pDest = nullptr;
    unsigned int* pSumms;

    for(int y = 0; y < height; ++y)
    {
      if(y % scaleFactor == 0)
      {
        pDest = destImage[y / scaleFactor];
      }
      pSrc = srcImage[y];
      pSumms = summs;
      for(int x = 0; x < width; x += scaleFactor, ++pSumms)
      {
        for(int i = 0; i < scaleFactor; ++i, ++pSrc)
        {
          *pSumms += *pSrc;
        }
      }

      if(y % scaleFactor == scaleFactor - 1)
      {
        pSumms = summs;
        for(int i = 0; i < destImage.width; ++i, ++pSumms, ++pDest)
        {
          *pDest = static_cast<unsigned char>(*pSumms / averagedPixels);
        }
        memset(summs, 0, destImage.width * sizeof(unsigned int));
      }
    }

    delete[] summs;
  }

  template<bool avx>
  void veritcal8BitShrink16(const __m_auto_i* pSrc, const ptrdiff_t lineLength, const __m_auto_i* srcEnd, __m_auto_i*& pDest)
  {
    while(pSrc != srcEnd)
    {
      const __m_auto_i* pSrc1 = pSrc + lineLength;
      const __m_auto_i* pSrc2 = pSrc1 + lineLength;
      const __m_auto_i* pSrc3 = pSrc2 + lineLength;
      const __m_auto_i* pSrc4 = pSrc3 + lineLength;
      const __m_auto_i* pSrc5 = pSrc4 + lineLength;
      const __m_auto_i* pSrc6 = pSrc5 + lineLength;
      const __m_auto_i* pSrc7 = pSrc6 + lineLength;
      const __m_auto_i* pSrc8 = pSrc7 + lineLength;
      const __m_auto_i* pSrc9 = pSrc8 + lineLength;
      const __m_auto_i* pSrc10 = pSrc9 + lineLength;
      const __m_auto_i* pSrc11 = pSrc10 + lineLength;
      const __m_auto_i* pSrc12 = pSrc11 + lineLength;
      const __m_auto_i* pSrc13 = pSrc12 + lineLength;
      const __m_auto_i* pSrc14 = pSrc13 + lineLength;
      const __m_auto_i* pSrc15 = pSrc14 + lineLength;
      const __m_auto_i* const lineEnd = pSrc1;
      while(pSrc != lineEnd)
      {
        _mmauto_storet_si_all<true>(++pDest,
          _mmauto_avg_epu8(
            _mmauto_avg_epu8(
              _mmauto_avg_epu8(
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc), _mmauto_loadt_si_all<true>(++pSrc1)),
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc2), _mmauto_loadt_si_all<true>(++pSrc3))
              ),
              _mmauto_avg_epu8(
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc4), _mmauto_loadt_si_all<true>(++pSrc5)),
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc6), _mmauto_loadt_si_all<true>(++pSrc7))
              )
            ),
            _mmauto_avg_epu8(
              _mmauto_avg_epu8(
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc8), _mmauto_loadt_si_all<true>(++pSrc9)),
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc10), _mmauto_loadt_si_all<true>(++pSrc11))
              ),
              _mmauto_avg_epu8(
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc12), _mmauto_loadt_si_all<true>(++pSrc13)),
                _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc14), _mmauto_loadt_si_all<true>(++pSrc15))
              )
            )
          )
          );
      }
      pSrc = pSrc15;
    }
  }

  template<bool avx>
  void horizontal8BitShrink16(const __m_auto_i* pSrc, const __m_auto_i* srcEnd, __m_auto_i* pDest)
  {
    static const __m_auto_i c_1 = _mmauto_set1_epi8(1);
    while(pSrc < srcEnd)
    {
      const __m_auto_i p0 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p1 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p2 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p3 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p4 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p5 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p6 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p7 = _mmauto_loadt_si_all<true>(++pSrc);

      if(avx && pSrc >= srcEnd)
      {
        _mm_store_si128(reinterpret_cast<__m128i*>(++pDest),
          _mmauto_castsiauto_si128(
            _mmauto_correct_256op(
              _mmauto_packus_epi16(
                _mmauto_srli_epi16(
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_correct_256op(
                            _mmauto_hadd_epi16(
                              _mmauto_maddubs_epi16(p0, c_1),
                              _mmauto_maddubs_epi16(p1, c_1)
                            )
                          ),
                          _mmauto_correct_256op(
                            _mmauto_hadd_epi16(
                              _mmauto_maddubs_epi16(p2, c_1),
                              _mmauto_maddubs_epi16(p3, c_1)
                            )
                          )
                        )
                      ),
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_correct_256op(
                            _mmauto_hadd_epi16(
                              _mmauto_maddubs_epi16(p4, c_1),
                              _mmauto_maddubs_epi16(p5, c_1)
                            )
                          ),
                          _mmauto_correct_256op(
                            _mmauto_hadd_epi16(
                              _mmauto_maddubs_epi16(p6, c_1),
                              _mmauto_maddubs_epi16(p7, c_1)
                            )
                          )
                        )
                      )
                    )
                  ),
                  4
                ),
                _mmauto_setzero_si_all()
              )
            )
          )
        );
        break;
      }

      const __m_auto_i p8 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p9 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p10 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p11 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p12 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p13 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p14 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p15 = _mmauto_loadt_si_all<true>(++pSrc);

      _mmauto_storet_si_all<true>(++pDest,
        _mmauto_correct_256op(
          _mmauto_packus_epi16(
            _mmauto_srli_epi16(
              _mmauto_correct_256op(
                _mmauto_hadd_epi16(
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p0, c_1),
                          _mmauto_maddubs_epi16(p1, c_1)
                        )
                      ),
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p2, c_1),
                          _mmauto_maddubs_epi16(p3, c_1)
                        )
                      )
                    )
                  ),
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p4, c_1),
                          _mmauto_maddubs_epi16(p5, c_1)
                        )
                      ),
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p6, c_1),
                          _mmauto_maddubs_epi16(p7, c_1)
                        )
                      )
                    )
                  )
                )
              ),
              4
            ),
            _mmauto_srli_epi16(
              _mmauto_correct_256op(
                _mmauto_hadd_epi16(
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p8, c_1),
                          _mmauto_maddubs_epi16(p9, c_1)
                        )
                      ),
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p10, c_1),
                          _mmauto_maddubs_epi16(p11, c_1)
                        )
                      )
                    )
                  ),
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p12, c_1),
                          _mmauto_maddubs_epi16(p13, c_1)
                        )
                      ),
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p14, c_1),
                          _mmauto_maddubs_epi16(p15, c_1)
                        )
                      )
                    )
                  )
                )
              ),
              4
            )
          )
        )
        );
    }
  }

  template<bool avx> void shrinkGrayscale16x16SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 16 == 0);

    destImage.setResolution(width / 16, height / 16);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink16<avx>(pSrc, lineLength, srcEnd, pDest);

    ASSERT((width * (height / 16)) % sizeof(__m_auto_i) == 0);

    srcEnd = pDest;
    pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;
    horizontal8BitShrink16<avx>(pDest, srcEnd, pDest);
  }

  template<bool avx>
  void veritcal8BitShrink8(const __m_auto_i* pSrc, const ptrdiff_t lineLength, const __m_auto_i* srcEnd, __m_auto_i*& pDest)
  {
    while(pSrc != srcEnd)
    {
      const __m_auto_i* pSrc1 = pSrc + lineLength;
      const __m_auto_i* pSrc2 = pSrc1 + lineLength;
      const __m_auto_i* pSrc3 = pSrc2 + lineLength;
      const __m_auto_i* pSrc4 = pSrc3 + lineLength;
      const __m_auto_i* pSrc5 = pSrc4 + lineLength;
      const __m_auto_i* pSrc6 = pSrc5 + lineLength;
      const __m_auto_i* pSrc7 = pSrc6 + lineLength;
      const __m_auto_i* const lineEnd = pSrc1;
      while(pSrc != lineEnd)
      {
        _mmauto_storet_si_all<true>(++pDest,
          _mmauto_avg_epu8(
            _mmauto_avg_epu8(
              _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc), _mmauto_loadt_si_all<true>(++pSrc1)),
              _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc2), _mmauto_loadt_si_all<true>(++pSrc3))
            ),
            _mmauto_avg_epu8(
              _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc4), _mmauto_loadt_si_all<true>(++pSrc5)),
              _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc6), _mmauto_loadt_si_all<true>(++pSrc7))
            )
          )
          );
      }
      pSrc = pSrc7;
    }
  }

  template<bool avx>
  void horizontal8BitShrink8(const __m_auto_i* pSrc, const __m_auto_i* srcEnd, __m_auto_i* pDest)
  {
    static const __m_auto_i c_1 = _mmauto_set1_epi8(1);

    while(pSrc <= srcEnd)
    {
      const __m_auto_i p0 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p1 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p2 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p3 = _mmauto_loadt_si_all<true>(++pSrc);

      if(avx && pSrc == srcEnd)
      {
        _mm_store_si128(reinterpret_cast<__m128i*>(++pDest),
          _mmauto_castsiauto_si128(
            _mmauto_correct_256op(
              _mmauto_packus_epi16(
                _mmauto_srli_epi16(
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p0, c_1),
                          _mmauto_maddubs_epi16(p1, c_1)
                        )
                      ),
                      _mmauto_correct_256op(
                        _mmauto_hadd_epi16(
                          _mmauto_maddubs_epi16(p2, c_1),
                          _mmauto_maddubs_epi16(p3, c_1)
                        )
                      )
                    )
                  ),
                  3
                ),
                _mmauto_setzero_si_all()
              )
            )
          )
        );
        break;
      }

      const __m_auto_i p4 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p5 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p6 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p7 = _mmauto_loadt_si_all<true>(++pSrc);
      _mmauto_storet_si_all<true>(++pDest,
        _mmauto_correct_256op(
          _mmauto_packus_epi16(
            _mmauto_srli_epi16(
              _mmauto_correct_256op(
                _mmauto_hadd_epi16(
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_maddubs_epi16(p0, c_1),
                      _mmauto_maddubs_epi16(p1, c_1)
                    )
                  ),
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_maddubs_epi16(p2, c_1),
                      _mmauto_maddubs_epi16(p3, c_1)
                    )
                  )
                )
              ),
              3
            ),
            _mmauto_srli_epi16(
              _mmauto_correct_256op(
                _mmauto_hadd_epi16(
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_maddubs_epi16(p4, c_1),
                      _mmauto_maddubs_epi16(p5, c_1)
                    )
                  ),
                  _mmauto_correct_256op(
                    _mmauto_hadd_epi16(
                      _mmauto_maddubs_epi16(p6, c_1),
                      _mmauto_maddubs_epi16(p7, c_1)
                    )
                  )
                )
              ),
              3
            )
          )
        )
        );
    }
  }

  template<bool avx> void shrinkGrayscale8x8SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 8 == 0);

    destImage.setResolution(width / 8, height / 8);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink8<avx>(pSrc, lineLength, srcEnd, pDest);

    srcEnd = pDest;
    pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;
    horizontal8BitShrink8<avx>(pDest, srcEnd, pDest);
  }

  template<bool avx>
  void veritcal8BitShrink4(const __m_auto_i* pSrc, const ptrdiff_t lineLength, const __m_auto_i* srcEnd, __m_auto_i*& pDest)
  {
    while(pSrc != srcEnd)
    {
      const __m_auto_i* pSrc1 = pSrc + lineLength;
      const __m_auto_i* pSrc2 = pSrc1 + lineLength;
      const __m_auto_i* pSrc3 = pSrc2 + lineLength;
      const __m_auto_i* const lineEnd = pSrc1;
      while(pSrc != lineEnd)
      {
        _mmauto_storet_si_all<true>(++pDest,
          _mmauto_avg_epu8(
            _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc), _mmauto_loadt_si_all<true>(++pSrc1)),
            _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc2), _mmauto_loadt_si_all<true>(++pSrc3))
          )
          );
      }
      pSrc = pSrc3;
    }
  }

  template<bool avx>
  void horizontal8BitShrink4(const __m_auto_i* pSrc, const __m_auto_i* srcEnd, __m_auto_i* pDest)
  {
    static const __m_auto_i c_1 = _mmauto_set1_epi8(1);
    //ASSERT(static_cast<long long>(pDest) % sizeof(__m128i) == 0);
    //bool oneAdditionalSSE = false;
    //if(avx && pDest % sizeof(__m256i) == sizeof(__m128i))
    //{
    //  true;
    //  pDest = static_cast<__m_auto_i*>(--static_cast<__m128i*>(pDest));
    //}

    while(pSrc != srcEnd)
    {
      const __m_auto_i p0 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p1 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p2 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p3 = _mmauto_loadt_si_all<true>(++pSrc);
      _mmauto_storet_si_all<true>(++pDest,
        _mmauto_correct_256op(
          _mmauto_packus_epi16(
            _mmauto_srli_epi16(
              _mmauto_correct_256op(
                _mmauto_hadd_epi16(
                  _mmauto_maddubs_epi16(p0, c_1),
                  _mmauto_maddubs_epi16(p1, c_1)
                )
              ),
              2
            ),
            _mmauto_srli_epi16(
              _mmauto_correct_256op(
                _mmauto_hadd_epi16(
                  _mmauto_maddubs_epi16(p2, c_1),
                  _mmauto_maddubs_epi16(p3, c_1)
                )
              ),
              2
            )
          )
        )
        );
    }

    //if(oneAdditionalSSE)
    //{
    //  __m128i* pSrc128 = static_cast<__m128i>(pSrc);
    //  __m128i* pDest128 = static_cast<__m128i>(pDest);
    //  const __m128i p0 = _mmauto_loadt_si_all<true>(++pSrc128);
    //  const __m128i p1 = _mmauto_loadt_si_all<true>(++pSrc128);
    //  const __m128i p2 = _mmauto_loadt_si_all<true>(++pSrc128);
    //  const __m128i p3 = _mmauto_loadt_si_all<true>(++pSrc128);
    //  _mmauto_storet_si_all<true>(++pDest128,
    //    _mm_packus_epi16(
    //      _mm_srli_epi16(
    //        _mm_correct_256op(
    //          _mm_hadd_epi16(
    //            _mm_maddubs_epi16(p0, c_1),
    //            _mm_maddubs_epi16(p1, c_1)
    //          )
    //        ),
    //        2
    //      ),
    //      _mm_srli_epi16(
    //        _mm_correct_256op(
    //          _mm_hadd_epi16(
    //            _mm_maddubs_epi16(p2, c_1),
    //            _mm_maddubs_epi16(p3, c_1)
    //          )
    //        ),
    //        2
    //      )
    //    )
    //    );
    //}
  }

  template<bool avx> void shrinkGrayscale4x4SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 4 == 0);

    destImage.setResolution(width / 4, height / 4);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink4<avx>(pSrc, lineLength, srcEnd, pDest);

    srcEnd = pDest;
    pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;
    horizontal8BitShrink4<avx>(pDest, srcEnd, pDest);
  }

  template<bool avx>
  void veritcal8BitShrink2(const __m_auto_i* pSrc, const ptrdiff_t lineLength, const __m_auto_i* srcEnd, __m_auto_i*& pDest)
  {
    while(pSrc != srcEnd)
    {
      const __m_auto_i* pSrc1 = pSrc + lineLength;
      const __m_auto_i* const lineEnd = pSrc1;
      while(pSrc != lineEnd)
      {
        _mmauto_storet_si_all<true>(++pDest, _mmauto_avg_epu8(_mmauto_loadt_si_all<true>(++pSrc), _mmauto_loadt_si_all<true>(++pSrc1)));
      }
      pSrc = pSrc1;
    }
  }

  template<bool avx> void shrinkGrayscale2x2SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 2 == 0);

    destImage.setResolution(width / 2, height / 2);

    static const __m_auto_i c_1 = _mmauto_set1_epi8(1);

    const __m_auto_i* pSrc0 = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const __m_auto_i* pSrc1 = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - 1;
    const ptrdiff_t lineLength = pSrc1 - pSrc0;
    const __m_auto_i* const srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;
    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    if(width % (2 * sizeof(__m_auto_i)) == 0)
    {
      for(; pSrc0 != srcEnd; pSrc0 += lineLength, pSrc1 += lineLength)
      {
        const __m_auto_i* const lineEnd = pSrc1;
        while(pSrc0 != lineEnd)
        {
          const __m_auto_i p0 = _mmauto_loadt_si_all<true>(++pSrc0);
          const __m_auto_i p1 = _mmauto_loadt_si_all<true>(++pSrc0);
          const __m_auto_i p2 = _mmauto_loadt_si_all<true>(++pSrc1);
          const __m_auto_i p3 = _mmauto_loadt_si_all<true>(++pSrc1);

          _mmauto_storet_si_all<true>(++pDest,
            _mmauto_correct_256op(
              _mmauto_packus_epi16(
                _mmauto_srli_epi16(
                  _mmauto_avg_epu16(
                    _mmauto_maddubs_epi16(p0, c_1),
                    _mmauto_maddubs_epi16(p2, c_1)
                  ),
                  1
                ),
                _mmauto_srli_epi16(
                  _mmauto_avg_epu16(
                    _mmauto_maddubs_epi16(p1, c_1),
                    _mmauto_maddubs_epi16(p3, c_1)
                  ),
                  1
                )
              )
            )
            );
        }
      }
    }
    else
    {
      ASSERT((width * height) % (2 * sizeof(__m_auto_i)) == 0);
      for(; pSrc0 != srcEnd; pSrc0 += lineLength, pSrc1 += lineLength)
      {
        __m_auto_i const* lineEnd = pSrc1;
        while(pSrc0 != lineEnd)
        {
          const __m_auto_i p0 = _mmauto_loadt_si_all<true>(++pSrc0);
          const __m_auto_i p2 = _mmauto_loadt_si_all<true>(++pSrc1);
          if(pSrc0 == lineEnd)
          {
            pSrc0 += lineLength;
            pSrc1 += lineLength;
            lineEnd = pSrc1;
          }
          const __m_auto_i p1 = _mmauto_loadt_si_all<true>(++pSrc0);
          const __m_auto_i p3 = _mmauto_loadt_si_all<true>(++pSrc1);

          _mmauto_storet_si_all<true>(++pDest,
            _mmauto_correct_256op(
              _mmauto_packus_epi16(
                _mmauto_srli_epi16(
                  _mmauto_avg_epu16(
                    _mmauto_maddubs_epi16(p0, c_1),
                    _mmauto_maddubs_epi16(p2, c_1)
                  ),
                  1
                ),
                _mmauto_srli_epi16(
                  _mmauto_avg_epu16(
                    _mmauto_maddubs_epi16(p1, c_1),
                    _mmauto_maddubs_epi16(p3, c_1)
                  ),
                  1
                )
              )
            )
            );
        }
      }
    }
  }

  void shrinkColorChannelNxN(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage, unsigned int downScalesExponent)
  {
    const int scaleFactor = static_cast<int>(1 << downScalesExponent);
    const int width = srcImage.width;
#ifndef NDEBUG
    const int height = srcImage.height;
#endif // NDEBUG

    ASSERT(scaleFactor > 1);
    ASSERT(width % (scaleFactor >> 1) == 0);
    ASSERT(height % scaleFactor == 0);

    if(width % 16 == 0)
    {
      if(_supportsAVX2 && width / (scaleFactor >> 1) % 32 == 0)
      {
        switch(downScalesExponent)
        {
          case 1:
            shrinkColorChannel2x2SSE<true>(srcImage, destImage);
            return;
          case 2:
            shrinkColorChannel4x4SSE<true>(srcImage, destImage);
            return;
          case 3:
            shrinkColorChannel8x8SSE<true>(srcImage, destImage);
            return;
          case 4:
            shrinkColorChannel16x16SSE<true>(srcImage, destImage);
            return;
        }
      }
      else
      {
        switch(downScalesExponent)
        {
          case 1:
            shrinkColorChannel2x2SSE<false>(srcImage, destImage);
            return;
          case 2:
            shrinkColorChannel4x4SSE<false>(srcImage, destImage);
            return;
          case 3:
            shrinkColorChannel8x8SSE<false>(srcImage, destImage);
            return;
          case 4:
            shrinkColorChannel16x16SSE<false>(srcImage, destImage);
            return;
        }
      }
    }
  }

  template<bool avx> void shrinkColorChannel16x16SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 16 == 0);

    destImage.setResolution(width / 8, height / 16);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink16<avx>(pSrc, lineLength, srcEnd, pDest);

    ASSERT((width * (height / 8)) % sizeof(__m_auto_i) == 0);

    srcEnd = pDest;
    pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;
    horizontal8BitShrink8<avx>(pDest, srcEnd, pDest);
  }

  template<bool avx> void shrinkColorChannel8x8SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 8 == 0);

    destImage.setResolution(width / 4, height / 8);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink8<avx>(pSrc, lineLength, srcEnd, pDest);

    srcEnd = pDest;
    pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;
    horizontal8BitShrink4<avx>(pDest, srcEnd, pDest);
  }

  template<bool avx> void shrinkColorChannel4x4SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 4 == 0);

    destImage.setResolution(width / 2, height / 4);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink4<avx>(pSrc, lineLength, srcEnd, pDest);

    srcEnd = pDest;
    pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;
    pSrc = pDest;

    static const __m_auto_i c_1 = _mmauto_set1_epi8(1);

    while(pSrc != srcEnd)
    {
      const __m_auto_i p0 = _mmauto_loadt_si_all<true>(++pSrc);
      const __m_auto_i p1 = _mmauto_loadt_si_all<true>(++pSrc);

      _mmauto_storet_si_all<true>(++pDest,
        _mmauto_correct_256op(
          _mmauto_packus_epi16(
            _mmauto_srli_epi16(
              _mmauto_maddubs_epi16(p0, c_1),
              1
            ),
            _mmauto_srli_epi16(
              _mmauto_maddubs_epi16(p1, c_1),
              1
            )
          )
        )
        );
    }
  }

  template<bool avx> void shrinkColorChannel2x2SSE(const TImage<unsigned char>& srcImage, TImage<unsigned char>& destImage)
  {
    const int width = srcImage.width;
    const int height = srcImage.height;
    ASSERT(width % sizeof(__m_auto_i) == 0);
    ASSERT(height % 2 == 0);

    destImage.setResolution(width, height / 2);

    const __m_auto_i* pSrc = reinterpret_cast<const __m_auto_i*>(srcImage[0]) - 1;
    const ptrdiff_t lineLength = reinterpret_cast<const __m_auto_i*>(srcImage[1]) - reinterpret_cast<const __m_auto_i*>(srcImage[0]);
    const __m_auto_i* srcEnd = reinterpret_cast<const __m_auto_i*>(srcImage[height]) - 1;

    __m_auto_i* pDest = reinterpret_cast<__m_auto_i*>(destImage[0]) - 1;

    veritcal8BitShrink2<avx>(pSrc, lineLength, srcEnd, pDest);
  }
}
