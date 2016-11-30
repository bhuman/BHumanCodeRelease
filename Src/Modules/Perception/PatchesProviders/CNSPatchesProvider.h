/**
 * The file declare a module that creates image patches from the regions of the
 * image the CNSImageProvider considers important.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/ImagePatches.h"
#include "Representations/Perception/BallPercepts/BallRegions.h"

MODULE(CNSPatchesProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CNSRegions),
  REQUIRES(Image),
  PROVIDES(ImagePatches),
});

class CNSPatchesProvider : public CNSPatchesProviderBase
{
private:
  void update(ImagePatches& imagePatches);
};
