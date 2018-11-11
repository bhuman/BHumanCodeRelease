/**
 * @author Felix Thielke
 */

#include "ImagePatches.h"
#include "Tools/Debugging/DebugImages.h"

void ImagePatch::serialize(In* in, Out* out)
{
  ASSERT(width == 0 || start != nullptr);

  STREAM(offset);
  STREAM(width);
  STREAM(height);

  if(out)
    for(int y = 0; y < height; ++y)
    {
      out->write((*this)[y], width * sizeof(PixelTypes::YUYVPixel));
    }
  else
  {
    widthStep = width;
    if(start != nullptr && !isReference)
    {
      delete[] start;
    }
    start = new PixelTypes::YUYVPixel[width * height];
    in->read(start, width * height * sizeof(PixelTypes::YUYVPixel));
  }
}

void ImagePatch::reg()
{
  PUBLISH(reg);
  REG_CLASS(ImagePatch);
  REG(offset);
  REG(width);
  REG(height);
}

void ImagePatches::toImage(TImage<PixelTypes::YUYVPixel>& dest) const
{
  dest.setResolution(imageWidth, imageHeight);
  for(const ImagePatch& patch : patches)
  {
    for(int y = 0; y < patch.height; ++y)
    {
      memcpy(&dest[y + patch.offset.y()][patch.offset.x()], patch[y], patch.width * sizeof(PixelTypes::YUYVPixel));
    }
  }
}

void ImagePatches::toImage(TImage<PixelTypes::YUYVPixel>& dest, PixelTypes::YUYVPixel color) const
{
  dest.setResolution(imageWidth, imageHeight);
  std::fill<PixelTypes::YUYVPixel*, PixelTypes::YUYVPixel>(dest[0], dest[imageHeight], color);
  toImage(dest);
}

void ImagePatches::draw() const
{
  COMPLEX_IMAGE("imagePatches")
  {
    PixelTypes::YUYVPixel color;
    color.color = 0;
    toImage(debugImage, color);
    SEND_DEBUG_IMAGE("imagePatches", debugImage);
  }
}
