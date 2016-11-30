/**
 * @author Alexis Tsogias, Felix Thielke
 */

#include "ThumbnailProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/Resize.h"

MAKE_MODULE(ThumbnailProvider, cognitionInfrastructure)

void ThumbnailProvider::update(Thumbnail& thumbnail)
{
  thumbnail.grayscale = grayscale;
  thumbnail.scale = 1 << downScales;
  if(grayscale)
  {
    if(downScales == 0)
    {
      thumbnail.imageGrayscale = theECImage.grayscaled;
    }
    else
    {
      Resize::shrinkGrayscaleNxN(theECImage.grayscaled, thumbnail.imageGrayscale, downScales);
    }
  }
  else
  {
    switch(thumbnail.scale)
    {
      case 8:
        Resize::shrink8x8SSE(theImage, thumbnail.image);
        break;
      case 4:
        Resize::shrink4x4SSE(theImage, thumbnail.image);
        break;
      default:
        Resize::shrinkNxN(theImage, thumbnail.image, downScales);
    }
    thumbnail.compressedImage.compress(thumbnail.image);
  }
}
