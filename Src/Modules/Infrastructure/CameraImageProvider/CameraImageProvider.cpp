/**
 * @file CameraImageProvider.cpp
 *
 * This file implements a module that creates the CameraImage as a reference to
 * the Image representation.
 * It will be obsolete when CameraImage will have completely replaced Image.
 *
 * @author Felix Thielke
 */

#include "CameraImageProvider.h"

MAKE_MODULE(CameraImageProvider, cognitionInfrastructure)

void CameraImageProvider::update(CameraImage& theCameraImage)
{
  theCameraImage.setReference(theImage.width, theImage.height * 2, const_cast<Image::Pixel*>(theImage[0]), theImage.timeStamp);
}
