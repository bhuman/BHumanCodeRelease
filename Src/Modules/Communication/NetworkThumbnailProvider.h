/**
 * @author Felix Thielke
 */

#pragma once

#include "Representations/Communication/NetworkThumbnail.h"
#include "Representations/Communication/TeammateData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Tools/Module/Module.h"

MODULE(NetworkThumbnailProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(ECImage),
  REQUIRES(TeammateData),
  PROVIDES_WITHOUT_MODIFY(NetworkThumbnail),
  DEFINES_PARAMETERS(
  {,
    (unsigned short)(64) sendSize,
    (unsigned char)(64) thumbnailWidth,
  }),
});

class NetworkThumbnailProvider : public NetworkThumbnailProviderBase
{
private:
  mutable TImage<PixelTypes::GrayscaledPixel> netThumbnail;

  std::vector<unsigned char> thumbnail;
  size_t offset;
  unsigned char width;
  unsigned char height;

  void update(NetworkThumbnail& networkThumbnail);
  void transformImage();

public:
  NetworkThumbnailProvider() : offset(0)
  {
    thumbnail.reserve(1024);
  }
};
