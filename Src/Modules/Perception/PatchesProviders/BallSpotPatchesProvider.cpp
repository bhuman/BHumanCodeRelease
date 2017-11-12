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
    const int left = std::max(0, spot.x() - scanRegionDimension / 2) >> 1;
    const int right = std::min(theCameraInfo.width, spot.x() + scanRegionDimension / 2) >> 1;
    const int top = std::max(0, spot.y() - scanRegionDimension / 2) >> 1;
    const int bottom = std::min(theCameraInfo.height, spot.y() + scanRegionDimension / 2) >> 1;
    if(left < right && top < bottom)
      imagePatches.patches.emplace_back(
        theImage,
        Vector2s(static_cast<short>(left), static_cast<short>(top)),
        static_cast<short>(right - left),
        static_cast<short>(bottom - top)
      );
  }
}
