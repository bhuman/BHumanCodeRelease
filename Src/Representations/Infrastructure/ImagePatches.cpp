/**
 * @author Felix Thielke
 */

#include "ImagePatches.h"
#include "Tools/Debugging/DebugImages.h"

void ImagePatch::serialize(In* in, Out* out)
{
  ASSERT(width == 0 || start != nullptr);

  STREAM_REGISTER_BEGIN;
  STREAM(offset);
  STREAM(width);
  STREAM(height);
  STREAM(isFullsize);

  if(out)
    for(int y = 0; y < height; ++y)
    {
      out->write((*this)[y], width * sizeof(Image::Pixel));
    }
  else
  {
    widthStep = width;
    if(start != nullptr && !isReference)
    {
      delete[] start;
    }
    start = new Image::Pixel[width * height];
    in->read(start, width * height * sizeof(Image::Pixel));
  }
  STREAM_REGISTER_FINISH;
}

void ImagePatches::toImage(Image& dest) const
{
  dest.setResolution(imageWidth, imageHeight, dest.isFullSize);
  for(const ImagePatch& patch : patches)
  {
    for(int y = 0; y < patch.height; ++y)
    {
      memcpy(patch.isFullsize ? (&dest[(y >> 1) + patch.offset.y()][patch.offset.x()] + (y & 1) * dest.width) : &dest[y + patch.offset.y()][patch.offset.x()], patch[y], patch.width * sizeof(Image::Pixel));
    }
  }
}

void ImagePatches::toImage(Image& dest, unsigned color) const
{
  dest.setResolution(imageWidth, imageHeight, true);
  for(Image::Pixel* p = dest[0], *pEnd = dest[imageHeight]; p < pEnd; ++p)
  {
    p->color = color;
  }
  toImage(dest);
}

void ImagePatches::draw() const
{
  COMPLEX_IMAGE("imagePatches")
  {
    toImage(debugImage, 0);
    SEND_DEBUG_IMAGE("imagePatches", debugImage);
  }
}
