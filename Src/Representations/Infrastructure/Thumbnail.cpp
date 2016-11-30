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
    dest.setResolution(imageGrayscale.width * scale / 2, imageGrayscale.height * scale / 2, true);
    for(int y = 0; y < dest.height * 2; ++y)
    {
      Image::Pixel* pDest = dest[y / 2] + y % 2 * dest.width;
      for(const ThumbnailImageGrayscale::PixelType* pSrc = imageGrayscale[y / scale], *pEnd = pSrc + imageGrayscale.width; pSrc < pEnd; ++pSrc)
      {
        pixel.yCbCrPadding = pixel.y = *pSrc;
        for(int x = 0; x < scale / 2; ++x)
          *pDest++ = pixel;
      }
    }
  }
  else
  {
    compressedImage.uncompress(*(const_cast<ThumbnailImage*>(&image)));

    dest.setResolution(image.width * scale / 2, image.height * scale / 2, true);
    for(int y = 0; y < dest.height * 2; ++y)
    {
      Image::Pixel* pDest = dest[y / 2] + y % 2 * dest.width;
      for(const ThumbnailImage::PixelType* pSrc = image[y / scale], *pEnd = pSrc + image.width; pSrc < pEnd; ++pSrc)
      {
        pixel.yCbCrPadding = pixel.y = pSrc->y;
        pixel.cb = pSrc->cb;
        pixel.cr = pSrc->cr;
        for(int x = 0; x < scale / 2; ++x)
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
