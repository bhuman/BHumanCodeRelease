/**
 * @file OptionalCameraImageProvider.cpp
 *
 * This file implements a module that will send an optional image.
 * It will only contain an actual image in certain conditions, otherwise it will
 * be empty and can be handled accordingly.
 *
 * @author Ayleen Lührsen
 */

#include "OptionalCameraImageProvider.h"

MAKE_MODULE(OptionalCameraImageProvider);

void OptionalCameraImageProvider::update(OptionalCameraImage& theOptionalCameraImage)
{
  if(theRefereeDetectionRequest.detectReferee)
    theOptionalCameraImage.image = theCameraImage;
  else
    theOptionalCameraImage.image.reset();
}
