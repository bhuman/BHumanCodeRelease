/**
 * @file Modules/Infrastructure/LowFrameRateImageProvider.h
 * This file declares a module that provides an image that is only rarely updated.
 * The image is intended for logging purposes.
 * @author Arne Böckmann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(LowFrameRateImageProvider,
{,
  REQUIRES(Image),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  PROVIDES_WITHOUT_MODIFY(LowFrameRateImage),
  LOADS_PARAMETERS(
  {,
    (int) frameRate, /**< Frames per minute. */
  }),
});

class LowFrameRateImageProvider : public LowFrameRateImageProviderBase
{
public:
  LowFrameRateImageProvider() : lastUpdateTime(0), storeNextImage(false) {}
  void update(LowFrameRateImage& image);

private:
  void logCurrentImage(LowFrameRateImage& lowFrameRateImage);
  void updateImage(LowFrameRateImage& lfrImage) const;

  unsigned lastUpdateTime; /**< Time of last update. */
  bool storeNextImage;
};
