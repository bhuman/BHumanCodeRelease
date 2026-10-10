/**
 * The file declares a module that provides the description of a grid for scanning
 * the image. The grid resolution adapts to the camera perspective.
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"

MODULE(ScanGridProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBoundary),
  PROVIDES(ScanGrid),
  LOADS_PARAMETERS(
  {,
    (int) minVerticalStepSize, /**< The minimum pixel distance between two neighboring vertical scan lines. */
    (int) minHorizontalLowResStepSize, /**< The minimum pixel distance between two neighboring horizontal scan lines. */
    (int) minNumOfLowResScanLines, /**< The minimum number of scan lines for low resolution. */
    (float) lineWidthRatio, /**< The ratio of field line width that is sampled when scanning the image. */
    (float) objectWidthRatio, /**< The ratio of ball or penalty mark width that is at least sampled when scanning the image. */
  }),
});

class ScanGridProvider : public ScanGridProviderBase
{
  struct ImageCornersOnField {
    Vector2f leftOnField;
    Vector2f rightOnField;
    bool valid;
  };

  enum VerticalBoundary {
    UPPER,
    LOWER
  };

  void update(ScanGrid& scanGrid) override;

  /**
   * Compute the furthest point away that could be part of the field given an unknown own position.
   * @return
   */
  int calcFieldLimit() const;

  /**
   * Determine vertical sampling points of the grid.
   * @param scanGrid
   * @param lowerImageCornersOnField
   */
  void setFullResY(ScanGrid& scanGrid, ImageCornersOnField& lowerImageCornersOnField) const;

  /**
   * Determine vertical sampling points of the grid.
   * @param scanGrid
   */
  void setLowResHorizontalLines(ScanGrid& scanGrid) const;

  /**
   *
   * @param boundary Whether to calc upper or lower image border position on field
   * @return
   */
  ImageCornersOnField calcImageCornersOnField(VerticalBoundary boundary) const;

  /**
   *
   * @param scanGrid
   * @param lowerImageCornersOnField
   */
  void setVerticalLines(ScanGrid& scanGrid, ImageCornersOnField& lowerImageCornersOnField) const;

  /**
   *
   * @param scanGrid
   * @param y
   */
  void addLowResHorizontalLine(ScanGrid& scanGrid, int y) const;

  /**
   * Determine the horizontal scanLine left edge, excluding both areas outside the field and inside the robots body.
   * @param usedY The height of the scan line in the image.
   * @return The x-coordinate of where to start the scan
   */
  int horizontalLeftScanStop(int usedY) const;

  /**
   * Determine the horizontal scanLine right edge, excluding both areas outside the field and inside the robots body.
   * @param usedY The height of the scan line in the image.
   * @return The x-coordinate of where to stop the scan
   */
  int horizontalRightScanStop(int usedY) const;

  /**
  * Determines the horizontal left scan start, excluding areas outside the field boundary
  * @param x x-coordinate from where to start searching for a start point.
  * @param y The height of the scan line in the image.
  * @return The x-coordinate of where to start the scan
  */
  int horizontalFieldBoundaryLeftScanStop(int x, int y) const;

  /**
   * Determines the horizontal right scan stop, excluding areas outside the field boundary
   * @param x-coordinate up to which to search for a stop point.
   * @param y The height of the scan line in the image.
   * @return The x-coordinate of where to stop the scan
   */
  int horizontalFieldBoundaryRightScanStop(int x, int y) const;
};
