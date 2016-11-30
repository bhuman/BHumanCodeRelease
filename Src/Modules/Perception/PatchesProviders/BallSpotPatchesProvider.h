/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/ImagePatches.h"

MODULE(BallSpotPatchesProvider,
{,
  REQUIRES(BallSpots),
  REQUIRES(CameraInfo),
  REQUIRES(Image),
  PROVIDES(ImagePatches),
});

class BallSpotPatchesProvider : public BallSpotPatchesProviderBase
{
private:
  void update(ImagePatches& imagePatches);
};
