/**
 * @file JPEGImage.cpp
 *
 * Implementation of struct JPEGImage
 */

#include "JPEGImage.h"
#include "Tools/ImageProcessing/SIMD.h"
#include "Platform/BHAssert.h"
#include "Platform/Memory.h"
#include <cstddef>
#include <jpeglib.h>

static boolean onDestEmpty(j_compress_ptr)
{
  FAIL("Unsupported operation.");
  return false;
}

static void onDestIgnore(j_compress_ptr) {}

static void onSrcSkip(j_decompress_ptr, long) {}

static boolean onSrcEmpty(j_decompress_ptr)
{
  FAIL("Unsupported operation.");
  return false;
}

static void onSrcIgnore(j_decompress_ptr) {}

JPEGImage::JPEGImage(const CameraImage& src)
{
  *this = src;
}

JPEGImage& JPEGImage::operator=(const CameraImage& src)
{
  allocator.resize(src.width * src.height * sizeof(CameraImage::PixelType));
  width = src.width;
  height = src.height / 2;
  timestamp = src.timestamp;

  jpeg_compress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);
  jpeg_create_compress(&cInfo);

  if(!cInfo.dest)
    cInfo.dest = static_cast<jpeg_destination_mgr*>(
                 (*cInfo.mem->alloc_small)(reinterpret_cast<j_common_ptr>(&cInfo), JPOOL_PERMANENT, sizeof(jpeg_destination_mgr)));
  cInfo.dest->init_destination = onDestIgnore;
  cInfo.dest->empty_output_buffer = onDestEmpty;
  cInfo.dest->term_destination = onDestIgnore;
  cInfo.dest->next_output_byte = static_cast<JOCTET*>(allocator.data());
  cInfo.dest->free_in_buffer = allocator.size();

  cInfo.image_width = width;
  cInfo.image_height = height * 2;
  cInfo.input_components = 4;
  cInfo.in_color_space = JCS_CMYK;
  cInfo.jpeg_color_space = JCS_CMYK;
  jpeg_set_defaults(&cInfo);
  cInfo.dct_method = JDCT_FASTEST;
  jpeg_set_quality(&cInfo, 75, true);

  jpeg_start_compress(&cInfo, true);

  while(cInfo.next_scanline < cInfo.image_height)
  {
    JSAMPROW rowPointer = const_cast<JSAMPROW>(reinterpret_cast<const unsigned char*>(src[0] + width * cInfo.next_scanline));
    jpeg_write_scanlines(&cInfo, &rowPointer, 1);
  }

  jpeg_finish_compress(&cInfo);
  size = unsigned((char unsigned*)cInfo.dest->next_output_byte - allocator.data());
  jpeg_destroy_compress(&cInfo);

  return *this;
}

void JPEGImage::toCameraImage(CameraImage& dest) const
{
  dest.setResolution(width, height * 2);
  dest.timestamp = timestamp;

  jpeg_decompress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);

  jpeg_create_decompress(&cInfo);

  if(!cInfo.src)
    cInfo.src = static_cast<jpeg_source_mgr*>(
                (*cInfo.mem->alloc_small)(reinterpret_cast<j_common_ptr>(&cInfo), JPOOL_PERMANENT, sizeof(jpeg_source_mgr)));
  cInfo.src->init_source       = onSrcIgnore;
  cInfo.src->fill_input_buffer = onSrcEmpty;
  cInfo.src->skip_input_data   = onSrcSkip;
  cInfo.src->resync_to_restart = jpeg_resync_to_restart;
  cInfo.src->term_source       = onSrcIgnore;
  cInfo.src->bytes_in_buffer   = allocator.size();
  cInfo.src->next_input_byte   = static_cast<const JOCTET*>(allocator.data());

  jpeg_read_header(&cInfo, true);
  jpeg_start_decompress(&cInfo);
  if(cInfo.num_components == 4) // full size images
  {
    // setup rows
    while(cInfo.output_scanline < cInfo.output_height)
    {
      JSAMPROW rowPointer = reinterpret_cast<unsigned char*>((dest[0] + width * cInfo.output_scanline));
      static_cast<void>(jpeg_read_scanlines(&cInfo, &rowPointer, 1));
    }
  }
  else
  {
    FAIL("Unsupported number of colors: " << cInfo.num_components << ".");
  }

  // finish decompress
  jpeg_finish_decompress(&cInfo);
  jpeg_destroy_decompress(&cInfo);
}

void JPEGImage::serialize(In* in, Out* out)
{
  STREAM(width);
  STREAM(height);

  timestamp |= 1 << 31;
  STREAM(timestamp);
  ASSERT((timestamp & 1 << 31) != 0);
  timestamp &= ~(1 << 31);

  STREAM(size);
  if(in)
  {
    allocator.resize(size);
    in->read(allocator.data(), size);
  }
  else
    out->write(allocator.data(), size);
}

void JPEGImage::reg()
{
  PUBLISH(reg);
  REG_CLASS(JPEGImage);
  REG(width);
  REG(height);
  REG(timestamp);
  REG(size);
}
