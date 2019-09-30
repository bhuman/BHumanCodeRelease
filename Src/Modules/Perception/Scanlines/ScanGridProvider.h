/**
 * The file declares a module that provides the description of a grid for scanning
 * the image. The grid resolution adapts to the camera perspective.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"

MODULE(ScanGridProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  PROVIDES(ScanGrid),
  DEFINES_PARAMETERS(
  {,
    (int)(3) minStepSize, /**< The minimum pixel distance between two neighboring scan lines. */
    (int)(25) minNumOfLowResScanLines, /**< The minimum number of scan lines for low resolution. */
    (float)(0.9f) lineWidthRatio, /**< The ratio of field line width that is sampled when scanning the image. */
    (float)(0.8f) ballWidthRatio, /**< The ratio of ball width that is sampled when scanning the image. */
  }),
});

class ScanGridProvider : public ScanGridProviderBase
{
  void update(ScanGrid& scanGrid) override;
};
