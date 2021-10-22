/**
 * @file ThumbnailProvider.h
 *
 * Declares a module which calculated a colored or gray-scaled thumbnail image.
 *
 * @author Alexis Tsogias
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"

MODULE(ThumbnailProvider,
{,
  REQUIRES(CameraImage),
  REQUIRES(ECImage),
  REQUIRES(CameraInfo),
  PROVIDES_WITHOUT_MODIFY(Thumbnail),
  LOADS_PARAMETERS(
  {,
    (unsigned) downScales,
    (bool) useUpperSize,
    (Thumbnail::Mode) mode,
  }),
});

class ThumbnailProvider : public ThumbnailProviderBase
{
  void update(Thumbnail& thumbnail) override;
};
