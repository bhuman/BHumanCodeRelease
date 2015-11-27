/**
 * @file JPEGImage.cpp
 *
 * Implementation of struct JPEGImage
 */

#include "Platform/BHAssert.h"
#include "JPEGImage.h"
#include "Tools/SIMD.h"
#include "Platform/SystemCall.h"
#include <cstddef>

JPEGImage::JPEGImage(const Image& image)
{
  *this = image;
}

JPEGImage& JPEGImage::operator=(const Image& src)
{
  setResolution(src.width, src.height, src.isFullSize);
  timeStamp = src.timeStamp;

  unsigned char* aiboAlignedImage = nullptr;
  if(!isFullSize)
  {
    aiboAlignedImage = (unsigned char*)SystemCall::alignedMalloc(width * height * 3, 16);
    toAiboAlignment((const unsigned char*)src[0], aiboAlignedImage);
  }

  jpeg_compress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);
  jpeg_create_compress(&cInfo);

  if(!cInfo.dest)
    cInfo.dest = (jpeg_destination_mgr*)
                 (*cInfo.mem->alloc_small)((j_common_ptr)&cInfo, JPOOL_PERMANENT, sizeof(jpeg_destination_mgr));
  cInfo.dest->init_destination = onDestIgnore;
  cInfo.dest->empty_output_buffer = onDestEmpty;
  cInfo.dest->term_destination = onDestIgnore;
  cInfo.dest->next_output_byte = (JOCTET*)(*this)[0];
  cInfo.dest->free_in_buffer = width * height;

  cInfo.image_width = width * (isFullSize ? 2 : 3);
  cInfo.image_height = height;
  cInfo.input_components = isFullSize ? 4 : 1;
  cInfo.in_color_space = JCS_GRAYSCALE;
  cInfo.jpeg_color_space = JCS_GRAYSCALE;
  jpeg_set_defaults(&cInfo);
  cInfo.dct_method = JDCT_FASTEST;
  jpeg_set_quality(&cInfo, 75, true);

  jpeg_start_compress(&cInfo, true);

  while(cInfo.next_scanline < cInfo.image_height)
  {
    JSAMPROW rowPointer = const_cast<JSAMPROW>(isFullSize ? (unsigned char*)src[cInfo.next_scanline] : &aiboAlignedImage[cInfo.next_scanline * cInfo.image_width]);
    jpeg_write_scanlines(&cInfo, &rowPointer, 1);
  }

  jpeg_finish_compress(&cInfo);
  size = unsigned((char unsigned*)cInfo.dest->next_output_byte - (unsigned char*)(*this)[0]);
  jpeg_destroy_compress(&cInfo);
  if(!isFullSize)
    SystemCall::alignedFree(aiboAlignedImage);

  return *this;
}

void JPEGImage::toImage(Image& dest) const
{
  dest.setResolution(width, height, isFullSize);
  dest.timeStamp = timeStamp;

  jpeg_decompress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);

  jpeg_create_decompress(&cInfo);

  if(!cInfo.src)
    cInfo.src = (jpeg_source_mgr*)
                (*cInfo.mem->alloc_small)((j_common_ptr) &cInfo, JPOOL_PERMANENT, sizeof(jpeg_source_mgr));
  cInfo.src->init_source       = onSrcIgnore;
  cInfo.src->fill_input_buffer = onSrcEmpty;
  cInfo.src->skip_input_data   = onSrcSkip;
  cInfo.src->resync_to_restart = jpeg_resync_to_restart;
  cInfo.src->term_source       = onSrcIgnore;
  cInfo.src->bytes_in_buffer   = (isFullSize ? 2 : 1) * width * height;
  cInfo.src->next_input_byte   = (const JOCTET*)(*this)[0];

  jpeg_read_header(&cInfo, true);
  jpeg_start_decompress(&cInfo);
  if(cInfo.num_components == 4) // old JPEG-compression (for compatibility with old logfiles)
  {
    // setup rows
    while(cInfo.output_scanline < cInfo.output_height)
    {
      JSAMPROW rowPointer = (unsigned char*)(dest[cInfo.output_scanline]);
      (void)jpeg_read_scanlines(&cInfo, &rowPointer, 1);
    }
  }
  else if(cInfo.num_components == 1) // new JPEG-compression
  {
    unsigned char* aiboAlignedImage = new unsigned char[width * height * 3];

    // setup rows
    while(cInfo.output_scanline < cInfo.output_height)
    {
      JSAMPROW rowPointer = &aiboAlignedImage[cInfo.output_scanline * cInfo.output_width];
      (void)jpeg_read_scanlines(&cInfo, &rowPointer, 1);
    }

    fromAiboAlignment(aiboAlignedImage, (unsigned char*)dest[0]);
    delete[] aiboAlignedImage;
  }
  else
  {
    ASSERT(false);
  }

  // finish decompress
  jpeg_finish_decompress(&cInfo);
  jpeg_destroy_decompress(&cInfo);
}

int JPEGImage::onDestEmpty(j_compress_ptr)
{
  ASSERT(false);
  return false;
}

void JPEGImage::onDestIgnore(j_compress_ptr) {}

void JPEGImage::onSrcSkip(j_decompress_ptr, long) {}

int JPEGImage::onSrcEmpty(j_decompress_ptr)
{
  ASSERT(false);
  return false;
}

void JPEGImage::onSrcIgnore(j_decompress_ptr) {}

void JPEGImage::toAiboAlignment(const unsigned char* src, unsigned char* dst) const
{
  ASSERT(width % 16 == 0);

  unsigned char mask[16] =
  {
    //0xFF, 0xFF, 0xFF, 0xFF,
    offsetof(Image::Pixel, cb), offsetof(Image::Pixel, cb) + 4, offsetof(Image::Pixel, cb) + 8, offsetof(Image::Pixel, cb) + 12,
    offsetof(Image::Pixel, y), offsetof(Image::Pixel, y) + 4, offsetof(Image::Pixel, y) + 8, offsetof(Image::Pixel, y) + 12,
    offsetof(Image::Pixel, cr), offsetof(Image::Pixel, cr) + 4, offsetof(Image::Pixel, cr) + 8, offsetof(Image::Pixel, cr) + 12,
    //0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF
  };

  const __m128i mMask = _mm_loadu_si128((__m128i*)&mask);

  const __m128i* pSrc;
  const __m128i* pSrcLineEnd;
  __m128i* pDst;

  __m128i p0;
  __m128i p1;
  __m128i p2;
  __m128i p3;
  __m128i mLowCbY;
  __m128i mHighCbY;
  __m128i mLowCr;
  __m128i mHighCr;

  for(int y = 0; y < height; ++y)
  {
    pSrc = (__m128i*)(src + y * widthStep * 4);
    pSrcLineEnd = (__m128i*)(src + y * widthStep * 4 + (width * 4));
    pDst = (__m128i*)(dst + y * width * 3);
    for(; pSrc < pSrcLineEnd; pSrc += 4, ++pDst)
    {
      p0 = _mm_loadu_si128(pSrc);     // yPadd1 cb1 y1 cr1 yPadd2 cb2 y2 cr2 yPadd3 cb3 y3 cr3 yPadd4 cb4 y4 cr4
      p1 = _mm_loadu_si128(pSrc + 1); // yPadd5 cb5 y5 cr5 yPadd6 cb6 y6 cr6 yPadd7 cb7 y7 cr7 yPadd8 cb8 y8 cr8
      p2 = _mm_loadu_si128(pSrc + 2); // yPadd9 cb9 y9 cr9 yPadd10 cb10 y10 cr10 yPadd11 cb11 y11 cr11 yPadd12 cb12 y12 cr12
      p3 = _mm_loadu_si128(pSrc + 3); // yPadd13 cb13 y13 cr13 yPadd14 cb14 y14 cr14 yPadd15 cb15 y15 cr15 yPadd16 cb16 y16 cr16

      p0 = SHUFFLE(p0, mMask); // cb1 cb2 cb3 cb4 y1 y2 y3 y4 cr1 cr2 cr3 cr4 0 0 0 0
      p1 = SHUFFLE(p1, mMask); // cb5 cb6 cb7 cb8 y5 y6 y7 y8 cr5 cr6 cr7 cr8 0 0 0 0
      p2 = SHUFFLE(p2, mMask); // cb9 cb10 cb11 cb12 y9 y10 y11 y12 cr9 cr10 cr11 cr12 0 0 0 0
      p3 = SHUFFLE(p3, mMask); // cb13 cb14 cb15 cb16 y13 y14 y15 y16 cr13 cr14 cr15 cr16 0 0 0 0

      mLowCbY = _mm_unpacklo_epi32(p0, p1);  // cb1 cb2 cb3 cb4 cb5 cb6 cb7 cb8 y1 y2 y3 y4 y5 y6 y7 y8
      mHighCbY = _mm_unpacklo_epi32(p2, p3); // cb9 cb10 cb11 cb12 cb13 cb14 cb15 cb16 y9 y10 y11 y12 y13 y14 y15 y16
      mLowCr = _mm_unpackhi_epi32(p0, p1);   // cr1 cr2 cr3 cr4 cr5 cr6 cr7 cr8 0 0 0 0 0 0 0 0
      mHighCr = _mm_unpackhi_epi32(p2, p3);  // cr9 cr10 cr11 cr12 cr13 cr14 cr15 cr16 0 0 0 0 0 0 0 0

      pDst[0] = _mm_unpacklo_epi64(mLowCbY, mHighCbY);
      pDst[width / 16] = _mm_unpackhi_epi64(mLowCbY, mHighCbY);
      pDst[width / 8] = _mm_unpacklo_epi64(mLowCr, mHighCr);
    }
  }
}

void JPEGImage::fromAiboAlignment(const unsigned char* src, unsigned char* dst) const
{
  const int resolutionWidth(this->width);
  for(int y = 0; y < height; y++)
  {
    const unsigned char* pSrc = src + y * resolutionWidth * 3;
    unsigned char* pDst = dst + y * widthStep * 4;
    for(int x = 0; x < resolutionWidth; x++)
    {
      pDst[1] = pSrc[0];
      pDst[0] = pDst[2] = pSrc[resolutionWidth];
      pDst[3] = pSrc[2 * resolutionWidth];
      pSrc++;
      pDst += 4;
    }
  }
}

void JPEGImage::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);

  if(isFullSize)
    timeStamp |= 1 << 31;
  STREAM(timeStamp);
  isFullSize = (timeStamp & 1 << 31) != 0;
  timeStamp &= ~(1 << 31);

  STREAM(size);
  if(in)
  {
    widthStep = 2 * width;
    in->read((*this)[0], size);
  }
  else
    out->write((*this)[0], size);
  STREAM_REGISTER_FINISH;
}
