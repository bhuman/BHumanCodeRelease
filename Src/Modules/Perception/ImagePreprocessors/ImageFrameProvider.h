/**
 * @file ImageFrameProvider.h
 *
 * This file declares a module that provides transformations from the time when the image was recorded.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/MotionControl/OdometryData.h"

MODULE(ImageFrameProvider,
{,
  REQUIRES(MotionOdometryData),
  PROVIDES(OdometryData),
});

class ImageFrameProvider : public ImageFrameProviderBase
{
  /**
   * This method updates the odometry data.
   * @param odometryData The updated representation.
   */
  void update(OdometryData& odometryData);
};
