/**
 * @author Alexis Tsogias
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Image.h"
#include <cstring>
#include "Platform/SystemCall.h"
#include "Platform/BHAssert.h"

struct Thumbnail : public Streamable
{
  template<typename Pixel>
  struct TImage : public Streamable
  {
    using PixelType = Pixel;
    static const int maxWidth = Image::maxResolutionWidth;
    static const int maxHeight = Image::maxResolutionHeight;
    int width = maxWidth;
    int height = maxHeight;

  private:
    static const int padding = 16;      /**< Some padding needed for the SSE instructions. */
    Pixel* imagePadding;                /**< A pointer to the memory for the image including some padding at the front and back. */
    Pixel* image;

  public:
    TImage();
    TImage(const TImage& other);
    virtual ~TImage();

    TImage& operator=(const TImage& other);

    Pixel* operator[](const int y)
    {
      return image + y * width;
    }

    const Pixel* operator[](const int y) const
    {
      return image + y * width;
    }

    void setResolution(int width, int height);

  private:
    virtual void serialize(In* in, Out* out);
  };

  template<typename Pixel, typename PixelUncompressed>
  struct TImageCompressed : public TImage<Pixel>
  {
    void compress(const TImage<PixelUncompressed>& uncompressedImage);
    void uncompress(TImage<PixelUncompressed>& uncompressedImage) const;
  };

  using ThumbnailImage = TImage<Image::Pixel>;
  using ThumbnailImageCompressed = TImageCompressed<unsigned short, Image::Pixel>;
  using ThumbnailImageGrayscale = TImage<unsigned char>;

  ThumbnailImage image;
  ThumbnailImageCompressed compressedImage;
  ThumbnailImageGrayscale imageGrayscale;

  int scale;
  bool grayscale;

public:
  void toImage(Image& dest) const;

private:
  virtual void serialize(In* in, Out* out);
};

template<typename Pixel>
Thumbnail::TImage<Pixel>::TImage() : 
  width(maxWidth), height(maxHeight)
{
  imagePadding = static_cast<Pixel*>(SystemCall::alignedMalloc(maxWidth * maxHeight * sizeof(Pixel) + (padding * 2), 16));
  image = imagePadding + (padding / sizeof(Pixel));
}

template<typename Pixel>
Thumbnail::TImage<Pixel>::TImage(const TImage& other) : 
  Thumbnail::TImage<Pixel>::TImage(),
  width(other.width), height(other.height)
{
  memcpy(image, other.image, maxWidth * maxHeight * sizeof(Pixel));
}

template<typename Pixel>
Thumbnail::TImage<Pixel>::~TImage()
{
  SystemCall::alignedFree(imagePadding);
}

template<typename Pixel>
Thumbnail::TImage<Pixel>& Thumbnail::TImage<Pixel>::operator=(const TImage& other)
{
  width = other.width;
  height = other.height;
  memcpy(image, other.image, maxWidth * maxHeight * sizeof(Pixel));
  return *this;
}

template<typename Pixel>
void Thumbnail::TImage<Pixel>::setResolution(int width, int height)
{
  ASSERT(width <= maxWidth);
  ASSERT(height <= maxHeight);
  this->width = width;
  this->height = height;
}

template<typename Pixel>
void Thumbnail::TImage<Pixel>::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);

  if(out)
    out->write(image, width * height * sizeof(Pixel));
  else
    in->read(image, width * height * sizeof(Pixel));

  STREAM_REGISTER_FINISH;
}

template<typename Pixel, typename PixelUncompressed>
void Thumbnail::TImageCompressed<Pixel, PixelUncompressed>::compress(const TImage<PixelUncompressed>& uncompressedImage)
{
  ASSERT(this->maxWidth >= uncompressedImage.width);
  ASSERT(this->maxHeight >= uncompressedImage.height);

  this->width = uncompressedImage.width;
  this->height = uncompressedImage.height;

  for(int y = 0; y < this->height; ++y)
  {
    for(int x = 0; x < this->width; ++x)
    {
      //  6 y   5 cb    5 cr
      const PixelUncompressed& p = uncompressedImage[y][x];
      unsigned short py = static_cast<unsigned short>(p.y >> 2 << 10);
      unsigned short pcb = static_cast<unsigned short>(p.cb >> 3 << 5);
      unsigned short pcr = static_cast<unsigned short>(p.cr >> 3);
      (*this)[y][x] = py | pcb | pcr;
    }
  }
}

template<typename Pixel, typename PixelUncompressed>
void Thumbnail::TImageCompressed<Pixel, PixelUncompressed>::uncompress(TImage<PixelUncompressed>& uncompressedImage) const
{
  ASSERT(uncompressedImage.maxWidth >= this->width);
  ASSERT(uncompressedImage.maxHeight >= this->height);

  uncompressedImage.width = this->width;
  uncompressedImage.height = this->height;

  for(int y = 0; y < this->height; ++y)
  {
    for(int x = 0; x < this->width; ++x)
    {
      const Pixel& comp = (*this)[y][x];
      PixelUncompressed& uncomp = uncompressedImage[y][x];
      uncomp.y = static_cast<unsigned char>(comp >> 10 << 2);
      uncomp.cb = static_cast<unsigned char>(comp >> 5 << 3);
      uncomp.cr = static_cast<unsigned char>(comp << 3);
    }
  }
}
