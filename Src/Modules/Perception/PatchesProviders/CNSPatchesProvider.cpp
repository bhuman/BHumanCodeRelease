/**
 * The file implements a module that creates image patches from the regions of the
 * image the CNSImageProvider considers important.
 *
 * @author Thomas Röfer
 */

#include "CNSPatchesProvider.h"

MAKE_MODULE(CNSPatchesProvider, perception)

void CNSPatchesProvider::update(ImagePatches& imagePatches)
{
  imagePatches.imageWidth = static_cast<short>(theImage.width);
  imagePatches.imageHeight = static_cast<short>(theImage.height);
  imagePatches.patches.clear();
  for(const Boundaryi& region : theCNSRegions.regions)
    imagePatches.patches.emplace_back(theImage,
                                      Vector2s(static_cast<short>(region.x.min >> 1),
                                               static_cast<short>(region.y.min >> 1)),
                                      region.x.getSize() >> 1, region.y.getSize() >> 1);
}
