/**
* @author Alexis Tsogias
*/

#include "ThumbnailProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/MMX.h"
#include <cmath>
#include <cstring>

MAKE_MODULE(ThumbnailProvider, Cognition Infrastructure)

ThumbnailProvider::ThumbnailProvider() {}

void ThumbnailProvider::update(Thumbnail& thumbnail)
{
  thumbnail.scale = static_cast<int>(::pow(2.0, downScales));
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

void ThumbnailProvider::shrinkNxN(const Image& srcImage, Thumbnail::ThumbnailImage& destImage)
{
  int scaleFactor = static_cast<int>(::pow(2.0, downScales));

  ASSERT(srcImage.width % scaleFactor == 0);
  ASSERT(srcImage.height % scaleFactor == 0);

  destImage.setResolution(srcImage.width / scaleFactor, srcImage.height / scaleFactor);
  int height = srcImage.height;
  int width = srcImage.width;
  int averagedPixels = scaleFactor * scaleFactor;

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
      for(int i = 0; i < scaleFactor ; ++i, ++pSrc)
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
        pDest->y = static_cast<char>(pSumms->y / averagedPixels);
        pDest->cb = static_cast<char>(pSumms->cb / averagedPixels);
        pDest->cr = static_cast<char>(pSumms->cr / averagedPixels);
      }
      memset(summs, 0, destImage.width * sizeof(pixelSum));
    }
  }

  delete[] summs;
}

void ThumbnailProvider::shrink8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage)
{
  int scaleFactor = 8;
  int averagedPixels = scaleFactor * scaleFactor;
  ASSERT(srcImage.width % scaleFactor == 0);
  ASSERT(srcImage.height % scaleFactor == 0);

  destImage.setResolution(srcImage.width / scaleFactor, srcImage.height / scaleFactor);
  int height = srcImage.height;
  int width = srcImage.width;

  static const __m128i zero = _mm_setzero_si128();
  __m128i* summs = reinterpret_cast<__m128i*>(SystemCall::alignedMalloc(16 * destImage.width, 16));
  memset(summs, 0, destImage.width * 16);

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
        short* ptr = reinterpret_cast<short*>(pSumms);

        short sumY = ptr[offsetof(Image::Pixel, y)] + ptr[offsetof(Image::Pixel, y) + sizeof(Image::Pixel)];
        short sumCb = ptr[offsetof(Image::Pixel, cb)] + ptr[offsetof(Image::Pixel, cb) + sizeof(Image::Pixel)];
        short sumCr = ptr[offsetof(Image::Pixel, cr)] + ptr[offsetof(Image::Pixel, cr) + sizeof(Image::Pixel)];

        pDest->y = static_cast<char>(sumY / averagedPixels);
        pDest->cb = static_cast<char>(sumCb / averagedPixels);
        pDest->cr = static_cast<char>(sumCr / averagedPixels);
      }
      memset(summs, 0, destImage.width * 16);
    }
  }
  SystemCall::alignedFree(summs);
}

void ThumbnailProvider::shrink4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage)
{
  int scaleFactor = 4;
  int averagedPixels = scaleFactor * scaleFactor;
  ASSERT(srcImage.width % scaleFactor == 0);
  ASSERT(srcImage.height % scaleFactor == 0);

  destImage.setResolution(srcImage.width / scaleFactor, srcImage.height / scaleFactor);
  int height = srcImage.height;
  int width = srcImage.width;

  static const __m128i zero = _mm_setzero_si128();
  __m128i* summs = reinterpret_cast<__m128i*>(SystemCall::alignedMalloc(16 * destImage.width, 16));
  memset(summs, 0, destImage.width * 16);

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
        short* ptr = reinterpret_cast<short*>(pSumms);

        short sumY = ptr[offsetof(Image::Pixel, y)] + ptr[offsetof(Image::Pixel, y) + sizeof(Image::Pixel)];
        short sumCb = ptr[offsetof(Image::Pixel, cb)] + ptr[offsetof(Image::Pixel, cb) + sizeof(Image::Pixel)];
        short sumCr = ptr[offsetof(Image::Pixel, cr)] + ptr[offsetof(Image::Pixel, cr) + sizeof(Image::Pixel)];

        pDest->y = static_cast<char>(sumY / averagedPixels);
        pDest->cb = static_cast<char>(sumCb / averagedPixels);
        pDest->cr = static_cast<char>(sumCr / averagedPixels);
      }
      memset(summs, 0, destImage.width * 16);
    }
  }
  SystemCall::alignedFree(summs);
}