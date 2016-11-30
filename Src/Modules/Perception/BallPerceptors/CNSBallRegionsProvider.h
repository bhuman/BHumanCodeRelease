/**
 * This file declares a module that calculates the regions around ball spots that must be scanned
 * for a ball detection.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/BallPrediction.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/BallPercepts/BallRegions.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Tools/Module/Module.h"

MODULE(CNSBallRegionsProvider,
{,
  REQUIRES(BallPrediction),
  REQUIRES(BallSpots),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(BallRegions),
  PROVIDES(BallRegions),
  PROVIDES(CNSRegions),
  LOADS_PARAMETERS(
  {,
    (bool) usePrediction, /**< Include block around predicted ball position. */
    (float) sizeFactor, /**< Factor to increase search space around ball spot radius. */
    (float) predictedFactor, /**< Factor to increase search space around the predicted ball position. */
    (int) blockSizeX, /**< Horizontal block size. Must be a multiple of 16. */
    (int) blockSizeY, /**< Vertical block size. Must be a multiple of 16. */
  }),
});

class CNSBallRegionsProvider : public CNSBallRegionsProviderBase
{
  /**
   * Marks the blockSizeX*blockSizeY cells the CNS should be computed for.
   * There is a one block border around the actual image, i.e. the upper left corner is at (1, 1).
   */
  char searchGrid[32][42];
  std::vector<Vector2i> stack; /**< The stack used by the flood fill algorithm to group blocks. */

  void update(BallRegions& ballRegions);
  void update(CNSRegions& cnsRegions);

  void floodFill(int x, int y, Rangei& xRange, Rangei& yRange);

public:
  CNSBallRegionsProvider() {stack.reserve(sizeof(searchGrid));}
};
