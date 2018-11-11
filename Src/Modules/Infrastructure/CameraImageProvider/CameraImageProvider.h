/**
 * @file CameraImageProvider.h
 *
 * This file declares a module that creates the CameraImage as a reference to
 * the Image representation.
 * It will be obsolete when CameraImage will have completely replaced Image.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Tools/Module/Module.h"

MODULE(CameraImageProvider,
{,
  REQUIRES(Image),
  PROVIDES(CameraImage),
});

class CameraImageProvider : public CameraImageProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraImage The representation updated.
   */
  void update(CameraImage& theCameraImage) override;
};
