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
#include "Representations/Perception/GoalPercept.h"

MODULE(LowFrameRateImageProvider,
{,
  REQUIRES(Image),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  REQUIRES(GoalPercept),
  PROVIDES_WITHOUT_MODIFY(LowFrameRateImage),
  LOADS_PARAMETERS(
  {,
    (int) frameRate, /**< Frames per second. */
    (bool) perceptsOnly, /**<If true only images containing percepts will be logged */
    (bool) logGoalPercept, /**< If true images containing goal percepts will be logged. Only if perceptsOnly is true as well */
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
