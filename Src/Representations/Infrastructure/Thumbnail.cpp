/**
 * @author Alexis Tsogias
 */

#include "Thumbnail.h"

void Thumbnail::toImage(Image& dest) const
{
  Image::Pixel pixel;
  if(grayscale)
  {
    pixel.cb = pixel.cr = 127;
    dest.setResolution(imageGrayscale.width * scale, imageGrayscale.height * scale);
    for(int y = 0; y < dest.height; ++y)
    {
      Image::Pixel* pDest = dest[y];
      for(const ThumbnailImageGrayscale::PixelType* pSrc = imageGrayscale[y / scale], *pEnd = pSrc + imageGrayscale.width; pSrc < pEnd; ++pSrc)
      {
        pixel.y = *pSrc;
        for(int x = 0; x < scale; ++x)
          *pDest++ = pixel;
      }
    }
  }
  else
  {
    compressedImage.uncompress(*(const_cast<ThumbnailImage*>(&image)));
    dest.setResolution(image.width * scale, image.height * scale);
    for(int y = 0; y < dest.height; ++y)
    {
      Image::Pixel* pDest = dest[y];
      for(const ThumbnailImage::PixelType* pSrc = image[y / scale], *pEnd = pSrc + image.width; pSrc < pEnd; ++pSrc)
      {
        pixel.y = pSrc->y;
        pixel.cb = pSrc->cb;
        pixel.cr = pSrc->cr;
        for(int x = 0; x < scale; ++x)
          *pDest++ = pixel;
      }
    }
  }
}

void Thumbnail::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(grayscale);
  STREAM(scale);
  if(grayscale)
  {
    STREAM(imageGrayscale);
  }
  else
  {
    STREAM(compressedImage);
  }
  STREAM_REGISTER_FINISH;
}
