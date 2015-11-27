/**
* @author Alexis Tsogias
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Thumbnail.h"

MODULE(ThumbnailProvider,
{,
  REQUIRES(Image),
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

private:
  void shrinkNxN(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);
  void shrink8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);
  void shrink4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);

  void shrinkGrayscaleNxN(const Image& srcImage, Thumbnail::ThumbnailImageGrayscale& destImage);
  void shrinkGrayscale8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImageGrayscale& destImage);
  void shrinkGrayscale4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImageGrayscale& destImage);
};
