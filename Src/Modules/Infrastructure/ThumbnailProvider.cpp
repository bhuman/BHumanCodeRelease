/**
* @author Alexis Tsogias
*/

#include "ThumbnailProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/SIMD.h"
#include <cstring>
#include <cstddef>
#include <cmath>

MAKE_MODULE(ThumbnailProvider, cognitionInfrastructure)

void ThumbnailProvider::update(Thumbnail& thumbnail)
{
  thumbnail.grayscale = grayscale;
  thumbnail.scale = static_cast<int>(std::pow(2.0, downScales));
  if(grayscale)
  {
    switch(thumbnail.scale)
    {
      case 8:
        shrinkGrayscale8x8SSE(theImage, thumbnail.imageGrayscale);
        break;
      case 4:
        shrinkGrayscale4x4SSE(theImage, thumbnail.imageGrayscale);
        break;
      default:
        shrinkGrayscaleNxN(theImage, thumbnail.imageGrayscale);
    }
  }
  else
  {
    switch(thumbnail.scale)
    {
      case 8:
        shrink8x8SSE(theImage, thumbnail.image);
        break;
      case 4:
        shrink4x4SSE(theImage, thumbnail.image);
        break;
      default:
        shrinkNxN(theImage, thumbnail.image);
    }
    thumbnail.compressedImage.compress(thumbnail.image);
  }
}

void ThumbnailProvider::shrinkNxN(const Image& srcImage, Thumbnail::ThumbnailImage& destImage)
{
  const int scaleFactor = static_cast<int>(std::pow(2.0, downScales));;
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
  Thumbnail::ThumbnailImage::PixelType* pDest;
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

void ThumbnailProvider::shrink8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage)
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
  __m128i* summs = reinterpret_cast<__m128i*>(SystemCall::alignedMalloc(summsSize, 16));
  memset(summs, 0, summsSize);

  const Image::Pixel* pSrc;
  Thumbnail::ThumbnailImage::PixelType* pDest;
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
  SystemCall::alignedFree(summs);
}

void ThumbnailProvider::shrink4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage)
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
  __m128i* summs = reinterpret_cast<__m128i*>(SystemCall::alignedMalloc(summsSize, 16));
  memset(summs, 0, summsSize);

  const Image::Pixel* pSrc;
  Thumbnail::ThumbnailImage::PixelType* pDest;
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
  SystemCall::alignedFree(summs);
}

void ThumbnailProvider::shrinkGrayscaleNxN(const Image& srcImage, Thumbnail::ThumbnailImageGrayscale& destImage)
{
  const int scaleFactor = static_cast<int>(std::pow(2.0, downScales));;
  const int averagedPixels = scaleFactor * scaleFactor;
  const int width = srcImage.width;
  const int height = srcImage.height;

  ASSERT(width % scaleFactor == 0);
  ASSERT(height % scaleFactor == 0);

  destImage.setResolution(width / scaleFactor, height / scaleFactor);

  unsigned int* summs = new unsigned int[destImage.width];
  memset(summs, 0, destImage.width * sizeof(unsigned int));

  const Image::Pixel* pSrc;
  Thumbnail::ThumbnailImageGrayscale::PixelType* pDest;
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
        *pSumms += pSrc->y;
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

void ThumbnailProvider::shrinkGrayscale8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImageGrayscale& destImage)
{
  union
  {
    __m128i a;
    long long b[2];
  } splitter;

  const int scaleFactor = 8;
  const int width = srcImage.width;
  const int height = srcImage.height;

  ASSERT(width % scaleFactor == 0);
  ASSERT(height % scaleFactor == 0);

  destImage.setResolution(width / scaleFactor, height / scaleFactor);

  const unsigned char offset = offsetof(Image::Pixel, y);
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
  const __m128i mMask = _mm_loadu_si128(reinterpret_cast<__m128i*>(mask));

  const __m128i zero = _mm_setzero_si128();
  const int summsSize = destImage.width * 16;
  __m128i* summs = reinterpret_cast<__m128i*>(SystemCall::alignedMalloc(summsSize, 16));
  memset(summs, 0, summsSize);

  const Image::Pixel* pSrc;
  Thumbnail::ThumbnailImageGrayscale::PixelType* pDest;
  __m128i* pSumms;

  __m128i p0;
  __m128i p1;
  __m128i p2;
  __m128i p3;
  __m128i p4;
  __m128i p5;
  __m128i p6;
  __m128i p7;

  for(int y = 0; y < height; ++y)
  {
    if(y % scaleFactor == 0)
    {
      pDest = destImage[y / scaleFactor];
    }
    pSrc = srcImage[y];
    pSumms = summs;
    for(int x = 0; x < width; x += 8, pSrc += 8, ++pSumms)
    {
      p0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc));
      p1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc + 4));

      p0 = SHUFFLE(p0, mMask); // y0 y1 y2 y3 0 0 0 0 0 0 0 0 0 0 0 0
      p1 = SHUFFLE(p1, mMask); // y4 y5 y6 y7 0 0 0 0 0 0 0 0 0 0 0 0

      p0 = _mm_unpacklo_epi32(p0, p1); // y0 y1 y2 y3 y4 y5 y6 y7 0 0 0 0 0 0 0 0
      p0 = _mm_unpacklo_epi8(p0, zero); // y0 y1 y2 y3 y4 y5 y6 y7
      *pSumms = _mm_add_epi16(*pSumms, p0);
    }

    if(y % scaleFactor == scaleFactor - 1)
    {
      pSumms = summs;
      for(int i = 0; i < destImage.width; i += 8, pSumms += 8, pDest += 8)
      {
        p0 = *pSumms;
        p1 = *(pSumms + 1);
        p2 = *(pSumms + 2);
        p3 = *(pSumms + 3);
        p4 = *(pSumms + 4);
        p5 = *(pSumms + 5);
        p6 = *(pSumms + 6);
        p7 = *(pSumms + 7);

        p0 = _mm_hadd_epi16(p0, p1);
        p1 = _mm_hadd_epi16(p2, p3);
        p2 = _mm_hadd_epi16(p4, p5);
        p3 = _mm_hadd_epi16(p6, p7);
        p0 = _mm_hadd_epi16(p0, p1);
        p1 = _mm_hadd_epi16(p2, p3);
        p0 = _mm_hadd_epi16(p0, p1);
        p0 = _mm_srli_epi16(p0, 6);
        p0 = _mm_packus_epi16(p0, zero);
        splitter.a = p0;
        *reinterpret_cast<long long*>(pDest) = splitter.b[0];
      }
      memset(summs, 0, summsSize);
    }
  }
  SystemCall::alignedFree(summs);
}

void ThumbnailProvider::shrinkGrayscale4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImageGrayscale& destImage)
{
  union
  {
    __m128i a;
    long long b[2];
  } splitter;

  const int scaleFactor = 4;
  const int width = srcImage.width;
  const int height = srcImage.height;

  ASSERT(width % scaleFactor == 0);
  ASSERT(height % scaleFactor == 0);

  destImage.setResolution(width / scaleFactor, height / scaleFactor);

  const unsigned char offset = offsetof(Image::Pixel, y);
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
  const __m128i mMask = _mm_loadu_si128(reinterpret_cast<__m128i*>(mask));

  const __m128i zero = _mm_setzero_si128();
  const int summsSize = destImage.width * 16;
  __m128i* summs = reinterpret_cast<__m128i*>(SystemCall::alignedMalloc(summsSize, 16));
  memset(summs, 0, summsSize);

  const Image::Pixel* pSrc;
  Thumbnail::ThumbnailImageGrayscale::PixelType* pDest;
  __m128i* pSumms;

  __m128i p0;
  __m128i p1;
  __m128i p2;
  __m128i p3;

  for(int y = 0; y < height; ++y)
  {
    if(y % scaleFactor == 0)
    {
      pDest = destImage[y / scaleFactor];
    }
    pSrc = srcImage[y];
    pSumms = summs;
    for(int x = 0; x < width; x += 8, pSrc += 8, ++pSumms)
    {
      p0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc));
      p1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc + 4));

      p0 = SHUFFLE(p0, mMask); // y0 y1 y2 y3 0 0 0 0 0 0 0 0 0 0 0 0
      p1 = SHUFFLE(p1, mMask); // y4 y5 y6 y7 0 0 0 0 0 0 0 0 0 0 0 0

      p0 = _mm_unpacklo_epi32(p0, p1); // y0 y1 y2 y3 y4 y5 y6 y7 0 0 0 0 0 0 0 0
      p0 = _mm_unpacklo_epi8(p0, zero); // y0 y1 y2 y3 y4 y5 y6 y7
      *pSumms = _mm_add_epi16(*pSumms, p0);
    }

    if(y % scaleFactor == scaleFactor - 1)
    {
      pSumms = summs;
      for (int i = 0; i < destImage.width; i += 8, pSumms += 4, pDest += 8)
      {
        p0 = *pSumms;
        p1 = *(pSumms + 1);
        p2 = *(pSumms + 2);
        p3 = *(pSumms + 3);

        p0 = _mm_hadd_epi16(p0, p1);
        p1 = _mm_hadd_epi16(p2, p3);
        p0 = _mm_hadd_epi16(p0, p1);
        p0 = _mm_srli_epi16(p0, 4);
        p0 = _mm_packus_epi16(p0, zero);
        splitter.a = p0;
        *reinterpret_cast<long long*>(pDest) = splitter.b[0];
      }
      memset(summs, 0, summsSize);
    }
  }
  SystemCall::alignedFree(summs);
}
