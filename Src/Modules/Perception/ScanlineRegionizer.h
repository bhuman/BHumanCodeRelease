/**
 * The file declares a module that creates colored regions on a number of scanlines.
 * Although the scanlines are vertical, scanning is actually performed horizontally 
 * by advancing all vertical scanlines in parallel from the image bottom to the top.
 * The upper bound of all scanlines is the upper image border and the horizon.
 * The lower bound is the lower image border and the body contour.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ScanlineRegions.h"

MODULE(ScanlineRegionizer,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorTable),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES_WITH_DRAW(ScanlineRegions),
  DEFINES_PARAMETERS(
  {,
    (int)(40) numOfScanlines, /**< The number of vertical scan lines. */
    (float)(0.9f) lineWidthRatio, /**< The ratio of field line width that is sampled when scanning the image. */
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
  }),
});

class ScanlineRegionizer : public ScanlineRegionizerBase
{
private:
  struct ScanState
  {
    int y; /**< The lower y coordinate of the current region (exclusive). */
    ColorTable::Colors color; /**< The color of the current region. */
    int upperBodyLimit; /**< The upmost y coordinate of the body contour (inclusive). */
    bool isBelowBodyLimit; /**< Still scanning inside the body? */

    /**
     * Konstruktor.
     * @param y The lower y coordinate of the current region (exclusive).
     * @param color The color of the current region.
     * @param upperBodyLimit The upmost y coordinate of the body contour (inclusive).
     */
    ScanState(int y, const ColorTable::Colors& color, int upperBodyLimit)
    : y(y), color(color), upperBodyLimit(upperBodyLimit), isBelowBodyLimit(y > upperBodyLimit) {}
  };

  std::vector<ScanState> scanStates; /**< The states of the vertical scanlines while scanning. */

  /** Provides the representation. */
  void update(ScanlineRegions& regions);
};
