/**
 * @author Alexis Tsogias, Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"

MODULE(ThumbnailProvider,
{,
  REQUIRES(Image),
  REQUIRES(ECImage),
  REQUIRES(CameraInfo),
  PROVIDES_WITHOUT_MODIFY(Thumbnail),
  LOADS_PARAMETERS(
  {,
    (unsigned) downScales,
    (bool) useUpperSize,
    (bool) grayscale,
    (bool) saveColorWhenGrayscale,
  }),
});

class ThumbnailProvider : public ThumbnailProviderBase
{
public:
  void update(Thumbnail& thumbnail);
};
