/**
* @author Alexis Tsogias
*/

#include "Thumbnail.h"

void Thumbnail::draw() const
{
  compressedImage.uncompress(*(const_cast<ThumbnailImage*>(&image)));

  COMPLEX_DEBUG_IMAGE(thumbnailDI,
  {
    SET_DEBUG_IMAGE_SIZE(thumbnailDI, image.width * scale, image.height * scale);
    for(int y = 0; y < (image.height * scale); y += scale)
    {
      for(int x = 0; x < (image.width * scale); x += scale)
      {
        const ThumbnailImage::PixelType& pix = image[y / scale][x / scale];
        for(int i = 0; i < scale; ++i)
        {
          for(int j = 0; j < scale; ++j)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(thumbnailDI, x + i, y + j, pix.y , pix.cb , pix.cr);
          }
        }
      }
    }
    SEND_DEBUG_IMAGE(thumbnailDI);
  });
}

void Thumbnail::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(compressedImage);
  STREAM(scale);
  STREAM_REGISTER_FINISH;
}