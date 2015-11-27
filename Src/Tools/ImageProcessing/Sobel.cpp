/**
 * @file   Sobel.h
 *
 * A class with methods to perform sobel edge detection on images (optimized by SSE instructions).
 *
 * @author Alexis Tsogias
 */

#include <cstring>
#include <cstddef>
#include "Tools/SIMD.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Sobel.h"

Sobel::Image1D::Image1D(int maxWidth, int maxHeight) : maxWidth(maxWidth), maxHeight(maxHeight)
{
  width = maxWidth;
  height = maxHeight;
  yStart = 0;
  imagePadding = static_cast<Pixel*>(SystemCall::alignedMalloc(maxWidth * maxHeight * sizeof(Pixel) + (padding * 2), 16));
  image = imagePadding + padding;
}

Sobel::Image1D::Image1D(const Image1D& other) : maxWidth(other.maxWidth), maxHeight(other.maxHeight)
{
  width = other.width;
  height = other.height;
  yStart = other.yStart;
  imagePadding = static_cast<Pixel*>(SystemCall::alignedMalloc(maxWidth * maxHeight * sizeof(Pixel) + (2 * padding), 16));
  image = imagePadding + padding;
  memcpy(image, other.image, maxWidth * maxHeight * sizeof(Pixel));
}

Sobel::Image1D::~Image1D()
{
  SystemCall::alignedFree(imagePadding);
}

void Sobel::Image1D::setResolution(int width, int height)
{
  ASSERT(width <= maxWidth);
  ASSERT(height <= maxHeight);
  this->width = width;
  this->height = height;
}

Sobel::Image1D& Sobel::Image1D::operator=(const Image1D& other)
{
  ASSERT(maxWidth == other.maxWidth);
  ASSERT(maxHeight == other.maxHeight);
  width = other.width;
  height = other.height;
  yStart = other.yStart;
  memcpy(image, other.image, maxWidth * maxHeight * sizeof(Pixel));
  return *this;
}

Sobel::SobelImage::SobelImage(int maxWidth, int maxHeight) : maxWidth(maxWidth), maxHeight(maxHeight)
{
  width = maxWidth;
  height = maxHeight;
  yStart = 0;
  sImage = static_cast<SobelPixel*>(SystemCall::alignedMalloc(maxWidth * maxHeight * sizeof(SobelPixel), 16));
}

Sobel::SobelImage::SobelImage(const SobelImage& other) : maxWidth(other.maxWidth), maxHeight(other.maxHeight)
{
  width = other.width;
  height = other.height;
  yStart = other.yStart;
  sImage = static_cast<SobelPixel*>(SystemCall::alignedMalloc(maxWidth * maxHeight * sizeof(SobelPixel), 16));
  memcpy(sImage, other.sImage, maxWidth * maxHeight * sizeof(SobelPixel));
}

Sobel::SobelImage::~SobelImage()
{
  SystemCall::alignedFree(sImage);
}

void Sobel::SobelImage::setResolution(int width, int height)
{
  ASSERT(width <= maxWidth);
  ASSERT(height <= maxHeight);
  this->width = width;
  this->height = height;
}

Sobel::SobelImage& Sobel::SobelImage::operator=(const SobelImage& other)
{
  ASSERT(maxWidth == other.maxWidth);
  ASSERT(maxHeight == other.maxHeight);
  width = other.width;
  height = other.height;
  yStart = other.yStart;
  memcpy(sImage, other.sImage, maxWidth * maxHeight * sizeof(SobelPixel));
  return *this;
}

void Sobel::extractSingleChannelSSE(const Image& srcImage, Sobel::Image1D& destImage, Channel channel, int yStart)
{
  ASSERT(srcImage.width % 16 == 0);
  ASSERT(yStart >= 0);
  ASSERT(yStart <= srcImage.height);

  destImage.setResolution(srcImage.width, srcImage.height);
  destImage.yStart = yStart;

  unsigned char offset;

  switch(channel)
  {
    case Channel::Y:
      offset = offsetof(Image::Pixel, y);
      break;
    case Channel::Cb:
      offset = offsetof(Image::Pixel, cb);
      break;
    case Channel::Cr:
      offset = offsetof(Image::Pixel, cr);
      break;
    default:
      ASSERT(false);
  }

  unsigned char mask[16] =
  {
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF
  };
  mask[0] = offset;
  mask[1] = offset + 4;
  mask[2] = offset + 8;
  mask[3] = offset + 12;

  const __m128i mMask = _mm_loadu_si128(reinterpret_cast<__m128i*>(&mask));

  const __m128i* srcPixel;
  const __m128i* srcPixelLineEnd;
  __m128i* destPixel;

  __m128i p0;
  __m128i p1;
  __m128i p2;
  __m128i p3;

  int height = srcImage.height;

  destPixel = reinterpret_cast<__m128i*>(destImage[yStart]);
  for(int y = yStart; y < height; ++y)
  {
    for(srcPixel = reinterpret_cast<const __m128i*>(srcImage[y]), srcPixelLineEnd = reinterpret_cast<const __m128i*>(srcImage[y] + srcImage.width);
        srcPixel < srcPixelLineEnd; srcPixel += 4, ++destPixel)
    {
      p0 = _mm_loadu_si128(srcPixel);
      p1 = _mm_loadu_si128(srcPixel + 1);
      p2 = _mm_loadu_si128(srcPixel + 2);
      p3 = _mm_loadu_si128(srcPixel + 3);

      p0 = SHUFFLE(p0, mMask); // y0 y1 y2 y3 0 0 0 0 0 0 0 0 0 0 0 0
      p1 = SHUFFLE(p1, mMask); // y4 y5 y6 y7 0 0 0 0 0 0 0 0 0 0 0 0
      p2 = SHUFFLE(p2, mMask); // y8 y9 y10 y11 0 0 0 0 0 0 0 0 0 0 0 0
      p3 = SHUFFLE(p3, mMask); // y12 y13 y14 y15 0 0 0 0 0 0 0 0 0 0 0 0

      __m128i mLow = _mm_unpacklo_epi32(p0, p1); // y0 y1 y2 y3 y4 y5 y6 y7 0 0 0 0 0 0 0 0
      __m128i mHigh = _mm_unpacklo_epi32(p2, p3); // y8 y9 y10 y11 y12 y13 y14 y15 0 0 0 0 0 0 0 0

      *destPixel = _mm_unpacklo_epi64(mLow, mHigh);
    }
  }
}

void Sobel::sobelSSE(const Image1D& srcImage, SobelImage& destImage)
{
  ASSERT(srcImage.width % 16 == 0);
  ASSERT(srcImage.height >= 3);
  ASSERT(srcImage.yStart >= 0);
  ASSERT(srcImage.yStart <= srcImage.height);

  destImage.setResolution(srcImage.width, srcImage.height);
  destImage.yStart = srcImage.yStart;

  if(srcImage.yStart >= srcImage.height)
    return;

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
  for(pDestImg = reinterpret_cast<__m128i*>(destImage[destImage.yStart]), pDestImgLineEnd = reinterpret_cast<__m128i*>(destImage[destImage.yStart + 1]);
      pDestImg < pDestImgLineEnd; ++pDestImg)
  {
    *pDestImg = zeros;
  }


  int lastRow = destImage.height - 1;
  const Image1D::Pixel* p0 = srcImage[destImage.yStart];
  const Image1D::Pixel* p1 = srcImage[destImage.yStart + 1];
  const Image1D::Pixel* p2 = srcImage[destImage.yStart + 2];
  const Image1D::Pixel* p0LineEnd;

  for(int y = destImage.yStart + 1; y < lastRow; ++y)
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
  for(int y = destImage.yStart; y < destImage.height - 1; y++)
  {
    destImage[y]->index = 0;
    (destImage[y + 1] - 1)->index = 0;
  }
}
