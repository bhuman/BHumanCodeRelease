/**
* @author Alexis Tsogias
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(ThumbnailProvider)
  REQUIRES(Image)
  REQUIRES(CameraInfo)
  PROVIDES_WITH_OUTPUT_AND_DRAW(Thumbnail)
  DEFINES_PARAMETER(unsigned int, downScales, 3)
END_MODULE

class ThumbnailProvider : public ThumbnailProviderBase
{
public:
  ThumbnailProvider();

  void update(Thumbnail& thumbnail);

private:
  void shrinkNxN(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);

  void shrink8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);
  void shrink4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);
};