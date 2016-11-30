/**
 * @author Felix Thielke
 */

#include "BallSpotPatchesProvider.h"

MAKE_MODULE(BallSpotPatchesProvider, perception)

void BallSpotPatchesProvider::update(ImagePatches& imagePatches)
{
  imagePatches.imageWidth = static_cast<short>(theImage.width);
  imagePatches.imageHeight = static_cast<short>(theImage.height);
  imagePatches.patches.clear();
  for(const Vector2i& spot : theBallSpots.ballSpots)
  {
    const int scanRegionDimension = theCameraInfo.camera == CameraInfo::Camera::lower ? 48 * spot.y() / 185 + 48 : std::max(2 * theImage.neuralNetImageRadius, 259 * spot.y() / 1000 - 3);
    const int offsetX = spot.x() - scanRegionDimension / 2, offsetY = spot.y() - scanRegionDimension / 2;
    imagePatches.patches.emplace_back(
      theImage,
      Vector2s(
        static_cast<short>(std::max(0, std::min(theImage.width - 1, offsetX))),
        static_cast<short>(std::max(0, std::min(theImage.height - 1, offsetY)))
      ),
      static_cast<short>(std::min(scanRegionDimension, theImage.width - offsetX)),
      static_cast<short>(std::min(scanRegionDimension, theImage.height - offsetY))
    );
  }
}
