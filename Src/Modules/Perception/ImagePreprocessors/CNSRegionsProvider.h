/**
 * This file declares a module that calculates the regions around interesting spots that must
 * be searched by CNS-based image processing modules.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Tools/Module/Module.h"

MODULE(CNSRegionsProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CNSPenaltyMarkRegions),
  PROVIDES(CNSRegions),
  LOADS_PARAMETERS(
  {,
    (int) blockSizeX, /**< Horizontal block size. Must be a multiple of 16. */
    (int) blockSizeY, /**< Vertical block size. Must be a multiple of 16. */
  }),
});

class CNSRegionsProvider : public CNSRegionsProviderBase
{
  /**
   * Marks the blockSizeX*blockSizeY cells the CNS should be computed for.
   * There is a one block border around the actual image, i.e. the upper left corner is at (1, 1).
   * The mapping from 2-D to the flat array is done dynamically.
   */
  std::vector<char> grid;
  std::vector<Vector2i> stack; /**< The stack used by the flood fill algorithm to group blocks. */

  void update(CNSRegions& cnsRegions) override;

  /**
   * Determines the size of connected region in the search grid.
   * @param searchGrid Accessor for search grid.
   * @param x The x coordinate of a point in the regions.
   * @param y The y coordinate of a point in the regions.
   * @param xRange The horizontal extent of the regions is reported here. The coordinates are
   *               zero-based, i.e. one less than the coordinate system in which x is provided.
   * @param xRange The vertical extent of the regions is reported here. The coordinates are
   *               zero-based, i.e. one less than the coordinate system in which y is provided.
   */
  void floodFill(const std::function<char&(int y, int x)>& searchGrid, int x, int y, Rangei& xRange, Rangei& yRange);
};
