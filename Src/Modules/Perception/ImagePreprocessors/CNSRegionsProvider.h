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
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(WorldModelPrediction),
  PROVIDES(BallRegions),
  PROVIDES(CNSRegions),
  LOADS_PARAMETERS(
  {,
    (bool) dontProcessUpperImageIfProjectedBallIsOutside, /**< Whether the upper image should be processed when the projected last ball is outside the view of the upper camera */
    (float) maxDistance, /**< The maximum distance to process in demo mode */
    (int) blockSizeX, /**< Horizontal block size. Must be a multiple of 16. */
    (int) blockSizeY, /**< Vertical block size. Must be a multiple of 16. */
  }),
});

class CNSRegionsProvider : public CNSRegionsProviderBase
{
  void update(BallRegions& ballRegions) override;
  void update(CNSRegions& cnsRegions) override;

  bool checkProcessingOfUpperImage();
};
