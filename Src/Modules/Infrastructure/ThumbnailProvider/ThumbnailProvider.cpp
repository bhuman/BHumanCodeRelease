/**
 * @file ThumbnailProvider.cpp
 *
 * Implements a module which calculated a colored or gray-scaled thumbnail image.

 * @author Felix Thielke
 */

#include "ThumbnailProvider.h"
#include "Tools/ImageProcessing/Resize.h"


MAKE_MODULE(ThumbnailProvider, infrastructure);

void ThumbnailProvider::update(Thumbnail& thumbnail)
{
  const unsigned int downScales = useUpperSize && theCameraInfo.camera == CameraInfo::Camera::lower && this->downScales != 0 ? this->downScales - 1 : this->downScales;
  thumbnail.mode = mode;
  thumbnail.scale = 1 << downScales;

  if(downScales == 0)
    thumbnail.imageY = theECImage.grayscaled;
  else
    Resize::shrinkY(downScales, theECImage.grayscaled, thumbnail.imageY);

  if(mode == Thumbnail::yuv)
    Resize::shrinkUV(downScales, theCameraImage, thumbnail.imageUV);
}
