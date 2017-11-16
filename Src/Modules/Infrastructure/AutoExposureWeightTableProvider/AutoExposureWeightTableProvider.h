/**
 * @file AutoExposureWeightTableProvider.h
 *
 * This fills the representation, that describes how the camera is using the image reagions to calculate the auto exposure.
 *
 * The idea is, to ignore all parts above the horizon and inside bodycontour, prefer configurating, boost cells with last ball detection.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/AutoExposureWeightTable.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"

MODULE(AutoExposureWeightTableProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ScanGrid),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(WorldModelPrediction),

  PROVIDES(AutoExposureWeightTable),
  LOADS_PARAMETERS(
  {,
    (bool)(true) useStaticConfigValues,
    (AutoExposureWeightTable) configuredAutoExposureWeightTable,
    (AutoExposureWeightTable) resetAutoExposureWeightTable,
    (unsigned char) factorReduceAllBeforeApplyBallPrediction,
  }),
});

class AutoExposureWeightTableProvider : public AutoExposureWeightTableProviderBase
{
public:
  void update(AutoExposureWeightTable& autoExposureWeightTable);
};
