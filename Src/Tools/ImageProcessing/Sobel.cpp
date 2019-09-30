/**
 * @file   Sobel.cpp
 *
 * @author Alexis Tsogias
 */

#include <cstring>
#include <cstddef>
#include "Tools/ImageProcessing/SIMD.h"
#include "Platform/BHAssert.h"
#include "Platform/Memory.h"
#include "Sobel.h"

void Sobel::sobelSSE(const Image1D& srcImage, SobelImage& destImage)
{
  ASSERT(srcImage.width % 16 == 0);
  ASSERT(srcImage.height >= 3);

  destImage.setResolution(srcImage.width, srcImage.height);

  // a b c    0 1 2
  // d e f    3 4 5
  // g h i    6 7 8
  __m128i valA, valB, valC, valD, valF, valG, valH, valI;
  __m128i sumX;
  __m128i sumY;
  __m128i tmp;

  __m128i zeros = _mm_setzero_si128();

  __m128i* pDestImg;
  __m128i* pDestImgLineEnd;

  // Fill top line
  for(pDestImg = reinterpret_cast<__m128i*>(destImage[0]), pDestImgLineEnd = reinterpret_cast<__m128i*>(destImage[1]);
      pDestImg < pDestImgLineEnd; ++pDestImg)
  {
    *pDestImg = zeros;
  }

  int lastRow = destImage.height - 1;
  const Image1D::PixelType* p0 = srcImage[0];
  const Image1D::PixelType* p1 = srcImage[1];
  const Image1D::PixelType* p2 = srcImage[2];
  const Image1D::PixelType* p0LineEnd;

  for(int y = 1; y < lastRow; ++y)
  {
    for(p0LineEnd = srcImage[y], pDestImg = reinterpret_cast<__m128i*>(destImage[y]); p0 < p0LineEnd;
        p0 += 16, p1 += 16, p2 += 16, pDestImg += 2)
    {
      // laod values
      valA = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p0 - 1));
      valB = _mm_load_si128(reinterpret_cast<const __m128i*>(p0));
      valC = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p0 + 1));

      valD = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p1 - 1));
      valF = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p1 + 1));

      valG = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p2 - 1));
      valH = _mm_load_si128(reinterpret_cast<const __m128i*>(p2));
      valI = _mm_loadu_si128(reinterpret_cast<const __m128i*>(p2 + 1));

      sumX = _mm_avg_epu8(valA, valG); // sumX = (a + g) / 2
      sumX = _mm_avg_epu8(sumX, valD); // sumX = (sumX + d) / 2
      sumX = _mm_avg_epu8(sumX, zeros); // sumX = sumX / 2 with average, because there is no 8 bit shift
      tmp = _mm_avg_epu8(valC, valI); // tmp = (c + i) / 2
      tmp = _mm_avg_epu8(tmp, valF); // tnp = (tmp + f) / 2
      tmp = _mm_avg_epu8(tmp, zeros); // tnp = tnp / 2 with average, because there is no 8 bit shift
      sumX = _mm_sub_epi8(sumX, tmp);

      sumY = _mm_avg_epu8(valA, valC); // sumX = (a + c) / 2
      sumY = _mm_avg_epu8(sumY, valB); // sumX = (sumX + b) / 2
      sumY = _mm_avg_epu8(sumY, zeros); // sumX = sumX / 2 with average, because there is no 8 bit shift
      tmp = _mm_avg_epu8(valG, valI); // tmp = (g + i) / 2
      tmp = _mm_avg_epu8(tmp, valH); // tnp = (tmp + h) / 2
      tmp = _mm_avg_epu8(tmp, zeros); // tnp = tnp / 2 with average, because there is no 8 bit shift
      sumY = _mm_sub_epi8(sumY, tmp);

      *pDestImg = _mm_unpacklo_epi8(sumX, sumY);
      *(pDestImg + 1) = _mm_unpackhi_epi8(sumX, sumY);
    }
  }

  // Fill bottom line
  for(pDestImg = reinterpret_cast<__m128i*>(destImage[destImage.height - 1]), pDestImgLineEnd = reinterpret_cast<__m128i*>(destImage[destImage.height]);
      pDestImg < pDestImgLineEnd; ++pDestImg)
  {
    *pDestImg = zeros;
  }

  // Fill right and left border
  for(unsigned int y = 0; y < destImage.height - 1; y++)
  {
    destImage[y]->index = 0;
    (destImage[y + 1] - 1)->index = 0;
  }
}
