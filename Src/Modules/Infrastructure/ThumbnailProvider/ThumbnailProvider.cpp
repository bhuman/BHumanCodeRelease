/**
 * @author Alexis Tsogias, Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "ThumbnailProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/Resize.h"

MAKE_MODULE(ThumbnailProvider, cognitionInfrastructure)

void ThumbnailProvider::update(Thumbnail& thumbnail)
{
  const int downScales = std::max(0, static_cast<int>(this->downScales) - (useUpperSize && theCameraInfo.camera == CameraInfo::Camera::lower ? 1 : 0));
  thumbnail.grayscale = grayscale;
  thumbnail.hasGrayscaleColorData = grayscale && saveColorWhenGrayscale;
  thumbnail.scale = 1 << downScales;
  if(grayscale)
  {
    if(downScales == 0)
    {
      thumbnail.imageGrayscale = theECImage.grayscaled;
      thumbnail.imageU = theECImage.ued;
      thumbnail.imageV = theECImage.ved;
    }
    else
    {
      Resize::shrinkGrayscaleNxN(theECImage.grayscaled, thumbnail.imageGrayscale, downScales);
      Resize::shrinkColorChannelNxN(theECImage.ued, thumbnail.imageU, downScales);
      Resize::shrinkColorChannelNxN(theECImage.ved, thumbnail.imageV, downScales);
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
