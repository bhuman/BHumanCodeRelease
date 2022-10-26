/**
 * This file declares a module that calculates the regions around interesting spots that must
 * be searched by CNS-based image processing modules.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Framework/Module.h"

MODULE(CNSRegionsProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CNSPenaltyMarkRegions),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(WorldModelPrediction),
  PROVIDES(BallRegions),
  PROVIDES(CNSRegions),
  LOADS_PARAMETERS(
  {,
    (int) lowerMode, /** 0: penalty mark, 1: whole image, everything else: no processing */
    (int) upperMode, /** 0: penalty mark, 1: whole image, everything else: no processing */
    (bool) dontProcessUpperImageIfProjectedBallIsOutside, /** Whether the upper image should be processed when the projected last ball is outside the view of the upper camera */
    (float) maxDistance, /** The maximum distance to process in demo mode */
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

  void update(BallRegions& ballRegions) override;
  void update(CNSRegions& cnsRegions) override;

  bool checkProcessingOfUpperImage();

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
