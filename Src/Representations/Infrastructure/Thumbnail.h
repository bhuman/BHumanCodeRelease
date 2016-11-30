/**
 * @author Alexis Tsogias
 */

#pragma once

#include "Image.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/Streams/Streamable.h"
#include "Platform/Memory.h"
#include "Platform/BHAssert.h"
#include <cstring>

struct Thumbnail : public Streamable
{
  template<typename Pixel>
  struct CompressedImage : public TImage<Pixel>
  {
    void compress(const TImage<Image::Pixel>& uncompressedImage);
    void uncompress(TImage<Image::Pixel>& uncompressedImage) const;
  };

  using ThumbnailImage = TImage<Image::Pixel>;
  using ThumbnailImageCompressed = CompressedImage<unsigned short>;
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
void Thumbnail::CompressedImage<Pixel>::compress(const TImage<Image::Pixel>& uncompressedImage)
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
      const Image::Pixel& p = uncompressedImage[y][x];
      unsigned short py = static_cast<unsigned short>(p.y >> 2 << 10);
      unsigned short pcb = static_cast<unsigned short>(p.cb >> 3 << 5);
      unsigned short pcr = static_cast<unsigned short>(p.cr >> 3);
      (*this)[y][x] = py | pcb | pcr;
    }
  }
}

template<typename Pixel>
void Thumbnail::CompressedImage<Pixel>::uncompress(TImage<Image::Pixel>& uncompressedImage) const
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
      Image::Pixel& uncomp = uncompressedImage[y][x];
      uncomp.y = static_cast<unsigned char>(comp >> 10 << 2);
      uncomp.cb = static_cast<unsigned char>(comp >> 5 << 3);
      uncomp.cr = static_cast<unsigned char>(comp << 3);
    }
  }
}
