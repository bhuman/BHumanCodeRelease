/**
 * @file OptionalCameraImageProvider.h
 *
 * This file declares a module that will send an optional image.
 * It will only contain an actual image in certain conditions, otherwise it will
 * be empty and can be handled accordingly.
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Perception/ImagePreprocessing/OptionalCameraImage.h"
#include "Representations/Perception/RefereePercept/OptionalImageRequest.h"

MODULE(OptionalCameraImageProvider,
{,
  REQUIRES(CameraImage),
  REQUIRES(OptionalImageRequest),
  PROVIDES(OptionalCameraImage),
});

class OptionalCameraImageProvider : public OptionalCameraImageProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOptionalCameraImage The representation updated.
   */
  void update(OptionalCameraImage& theOptionalCameraImage) override;
};
