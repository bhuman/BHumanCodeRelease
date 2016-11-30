/**
 * @author Felix Thielke
 */

#pragma once

#include "Image.h"
#include "Tools/Streams/Streamable.h"

struct ImagePatch : public Streamable
{
public:
  bool isReference;
  Image::Pixel* start;
  Vector2s offset;
  unsigned short width;
  unsigned short height;
  unsigned short widthStep;
  bool isFullsize;

  ImagePatch() : isReference(false), start(nullptr), offset((short)0, (short)0), width(0), height(0), widthStep(0), isFullsize(false) {}
  ImagePatch(const Image& image, const Vector2s& offset, const unsigned short width, const unsigned short height) : isReference(true), start(const_cast<Image::Pixel*>(&image[offset])), offset(offset), width(width), height(height * (image.isFullSize ? 2 : 1)), widthStep(static_cast<const unsigned short>(image.isFullSize ? image.width : image.widthStep)), isFullsize(image.isFullSize)
  {
    ASSERT(offset.x() >= 0 && offset.x() + width <= image.width);
    ASSERT(offset.y() >= 0 && offset.y() + height <= image.height);
  }
  ~ImagePatch()
  {
    if(start != nullptr && !isReference)
    {
      delete[] start;
    }
  }

  inline const Image::Pixel* operator[](const size_t y) const
  {
    return start + (y * widthStep);
  }

private:
  virtual void serialize(In* in, Out* out);
};

STREAMABLE(ImagePatches,
{
private:
  mutable Image debugImage;
public:
  void toImage(Image & dest) const;
  void toImage(Image & dest, unsigned color) const;
  void draw() const,

  (unsigned short) imageWidth,
  (unsigned short) imageHeight,
  (std::vector<ImagePatch>) patches,
});
