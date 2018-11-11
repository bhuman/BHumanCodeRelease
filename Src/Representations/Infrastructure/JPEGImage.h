/**
 * @file JPEGImage.h
 *
 * Declaration of struct JPEGImage
 */

#pragma once

#include "Representations/Infrastructure/Image.h"

#ifdef WINDOWS

// INT32 and FAR conflict with any other header files...
#define INT32 _INT32
#undef FAR

// "boolean" conflicts with "rpcndr.h", so we force "jpeglib.h" not to define boolean
#ifdef __RPCNDR_H__
#define HAVE_BOOLEAN
#endif

#include <jpeglib.h>

#undef INT32
#undef FAR

#else

extern "C"
{
#include <jpeglib.h>
}

#endif

/**
 * Definition of a struct for JPEG-compressed images.
 */
struct JPEGImage : public Image
{
private:
  unsigned size; /**< The size of the compressed image. */

public:
  JPEGImage() = default;

  /**
   * Constructs a JPEG image from an image.
   * @param src The image used as template.
   */
  JPEGImage(const Image& src);

  /**
   * Assignment operator.
   * @param src The image used as template.
   * @return The resulting JPEG image.
   */
  JPEGImage& operator=(const Image& src);

  /**
   * Uncompress image.
   * @param dest Will receive the uncompressed image.
   */
  void toImage(Image& dest) const;

private:
  //!@name Handlers for JPEG-compression
  //!@{
  static int onDestEmpty(j_compress_ptr);
  static void onDestIgnore(j_compress_ptr);

  static void onSrcSkip(j_decompress_ptr cInfo, long numBytes);
  static int onSrcEmpty(j_decompress_ptr);
  static void onSrcIgnore(j_decompress_ptr);
  //!@}

  /**
   * Convert image from Nao's alignment (YUV422) to Aibo's alignment (one channel per line)
   * destination is asserted to be allocated
   * @param src Pointer to the source image in Nao's alignment
   * @param dst Pointer to the destination image
   */
  void toAiboAlignment(const unsigned char* src, unsigned char* dst) const;

  /**
   * Convert image from Aibo's alignment (one channel per line) to Nao's alignment (YUV422)
   * destination is asserted to be allocated
   * @param src Pointer to the source image in Aibo's alignment
   * @param dst Pointer to the destination image
   */
  void fromAiboAlignment(const unsigned char* src, unsigned char* dst) const;

protected:
  void serialize(In* in, Out* out);

private:
  static void reg();
};
