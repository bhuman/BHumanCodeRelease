/**
 * @file ImageFrameProvider.cpp
 *
 * This file implements a module that provides transformations from the time when the image was recorded.
 *
 * @author Arne Hasselbring
 */

#include "ImageFrameProvider.h"

MAKE_MODULE(ImageFrameProvider);

void ImageFrameProvider::update(OdometryData& odometryData)
{
  odometryData = static_cast<const OdometryData&>(theMotionOdometryData);
}
