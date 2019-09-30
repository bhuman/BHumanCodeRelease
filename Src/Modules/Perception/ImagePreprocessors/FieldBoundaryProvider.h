/**
 * @file FieldBoundaryProvider.h
 *
 * This file declares a module that estimates the field boundary using
 * a RANSAC algorithm for a single straight line or two intersecting
 * straight lines.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Module/Module.h"

MODULE(FieldBoundaryProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorScanLineRegionsVertical),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(Odometer),
  REQUIRES(OtherFieldBoundary),
  PROVIDES(FieldBoundary),
  DEFINES_PARAMETERS(
  {,
    (float)(800.f) minDistance, /**< Boundary spots closer than this distance will be ignored (in mm). */
    (unsigned)(10) minNumberOfSpots, /**< The minimum number of valid spots to calculate a field boundary. */
    (int)(50) maxNumberOfIterations, /**< Up to how often does RANSAC iterate? */
    (int)(100) maxSquaredError, /**< Limit at which deviations of spots from the boundary saturate (in pixel^2).  */
    (int)(4) spotAbovePenaltyFactor, /**< A spot being above this boundary is this factor worse than being below. */
    (float)(0.1f) acceptanceRatio, /**< Which overall ratio of maxSquaredError is good enough to end the RANSAC? */
  }),
});

class FieldBoundaryProvider : public FieldBoundaryProviderBase
{
  /** A boundary spot both in image and field coordinates. */
  struct Spot
  {
    Vector2i inImage; /**< The spot in image coordinates. */
    Vector2f onField; /**< The spot in robot-relative field coordinates. */

    Spot() = default;
    Spot(const Vector2i& inImage, const Vector2f& onField) : inImage(inImage), onField(onField) {}
  };

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFieldBoundary The representation updated.
   */
  void update(FieldBoundary& theFieldBoundary) override;

  /**
   * Predict where the previous field boundary will be in the current image.
   * @param fieldBoundary The field boundary that is updated.
   */
  void predict(FieldBoundary& fieldBoundary) const;

  /**
   * Find boundary spots in the current image (actually on vertical scan lines).
   * @param fieldBoundary The previous field boundary projected to the current image.
   *                      If it is outside the current image, it might be used to
   *                      determine spots instead of searching them in the image.
   * @param spots The spots found are returned here.
   */
  void findSpots(const FieldBoundary& fieldBoundary, std::vector<Spot>& spots) const;

  /**
   * Return a weighted, squared, and saturated error between boundary spots and a
   * boundary line.
   * @param error The vertical pixel offest between spot and line. Positive if the
   *              spot is above the line.
   */
  int effectiveError(int error) const
  {
    return std::min(sqr(error), maxSquaredError) * (error < 0 ? 1 : spotAbovePenaltyFactor);
  }

  /**
   * Calculate the field boundary using the RANSAC approach. The method always constructs a
   * model from three sample points and considers a straight line between the first two points
   * or also a perpedicular line (in field coordinates) to the third point.
   * @param spots The boundary spots that are sampled.
   * @param model A model of two or three spots describing the single or two lines.
   */
  void calcBoundary(const std::vector<Spot>& spots, std::vector<Spot>& model) const;

  /**
   * Fills the representation.
   * @param model The model found.
   * @param fieldBoundary The field boundary that is filled.
   */
  void fillRepresentation(const std::vector<Spot>& model, FieldBoundary& fieldBoundary) const;
};
