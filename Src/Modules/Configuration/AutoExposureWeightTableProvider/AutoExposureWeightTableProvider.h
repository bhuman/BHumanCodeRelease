/**
 * @file AutoExposureWeightTableProvider.h
 *
 * This file declares a module that detemines how different areas of a camera image should be
 * weighted when computing the auto exposure. The auto exposure weight table consists of 5x5
 * weights for corresponding image areas. The module limits the area considered to a
 * definable distance from the robots. In addition, the robot's body is excluded. If the ball
 * is expected to be in the image, it will influence the exposure computation with a
 * configurable ratio.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/AutoExposureWeightTable.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Module/Module.h"

MODULE(AutoExposureWeightTableProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FrameInfo),
  REQUIRES(WorldModelPrediction),
  PROVIDES(AutoExposureWeightTable),
  LOADS_PARAMETERS(
  {,
    (float) distanceThreshold, /**< The maximum field distance of areas considered (in mm). */
    (int) ballValidDelay, /**< How long is the ball prioritized after its last detection (in ms)? */
    (float) ballWeightRatio, /**< The ratio between the summed up weights of ball areas and other areas. */
    (int) maxWeightsUpdatedPerFrame, /**< The maximum number of weights updated per frame. */
    (bool) useStaticTables, /**< Only provide the static tables defined below? */
    (Matrix5uc[CameraInfo::numOfCameras]) staticTables, /**< Static weights each in the range [0 .. 100]. */
  }),
});

class AutoExposureWeightTableProvider : public AutoExposureWeightTableProviderBase
{
  int nextCellToUpdate[CameraInfo::numOfCameras] = {0, 0};

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theAutoExposureWeightTable The representation updated.
   */
  void update(AutoExposureWeightTable& theAutoExposureWeightTable) override;
};
