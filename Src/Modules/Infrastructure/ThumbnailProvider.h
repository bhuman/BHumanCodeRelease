/**
 * @author Alexis Tsogias, Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"

MODULE(ThumbnailProvider,
{,
  REQUIRES(Image),
  REQUIRES(ECImage),
  PROVIDES_WITHOUT_MODIFY(Thumbnail),
  LOADS_PARAMETERS(
  {,
    (unsigned) downScales,
    (bool) grayscale,
  }),
});

class ThumbnailProvider : public ThumbnailProviderBase
{
public:
  void update(Thumbnail& thumbnail);
};
