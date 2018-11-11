/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/ImageProcessing/PixelTypes.h"

struct ImagePatch : public Streamable
{
public:
  bool isReference;
  PixelTypes::YUYVPixel* start;
  Vector2s offset;
  unsigned short width;
  unsigned short height;
  unsigned short widthStep;

  ImagePatch() : isReference(false), start(nullptr), offset((short)0, (short)0), width(0), height(0), widthStep(0) {}
  ImagePatch(const TImage<PixelTypes::YUYVPixel>& image, const Vector2s& offset, const unsigned short width, const unsigned short height) : isReference(true), start(const_cast<PixelTypes::YUYVPixel*>(&image[offset])), offset(offset), width(width), height(height), widthStep(static_cast<const unsigned short>(image.width))
  {
    ASSERT(offset.x() >= 0 && offset.x() + width <= static_cast<int>(image.width));
    ASSERT(offset.y() >= 0 && offset.y() + height <= static_cast<int>(image.height));
  }
  ~ImagePatch()
  {
    if(start != nullptr && !isReference)
    {
      delete[] start;
    }
  }

  inline const PixelTypes::YUYVPixel* operator[](const size_t y) const
  {
    return start + (y * widthStep);
  }

protected:
  void serialize(In* in, Out* out) override;

private:
  static void reg();
};

STREAMABLE(ImagePatches,
{
private:
  mutable TImage<PixelTypes::YUYVPixel> debugImage;
public:
  void toImage(TImage<PixelTypes::YUYVPixel>& dest) const;
  void toImage(TImage<PixelTypes::YUYVPixel>& dest, PixelTypes::YUYVPixel color) const;
  void draw() const,

  (unsigned short) imageWidth,
  (unsigned short) imageHeight,
  (std::vector<ImagePatch>) patches,
});
