/**
 * @file PenaltyMarkRegionsProvider.h
 *
 * This file declares a module that determines candidate regions for
 * penalty marks in the image. It provides regions in which should be
 * searched for the center as well as the regions that must be provided
 * in the CNS image for the search to work.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldFeatures/PenaltyMarkSpots.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Module.h"

MODULE(PenaltyMarkRegionsProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorScanLineRegionsVerticalClipped),
  REQUIRES(FieldDimensions),
  REQUIRES(ScanGrid),
  REQUIRES(PenaltyMarkRegions),
  PROVIDES(PenaltyMarkRegions),
  PROVIDES(PenaltyMarkSpots),
  PROVIDES(CNSPenaltyMarkRegions),
  DEFINES_PARAMETERS(
  {,
    (float)(2000.f) maxDistanceOnField, /**< The maximum distance in which penalty marks are detected. */
    (int)(3) regionExtensionFactor, /**< Region heights are extended by this value times the expected line width to better merge diagonal lines. */
    (float)(0.5f) sizeToleranceRatio, /**< Acceptable deviation of the measured size from the expected one. */
    (float)(0.9f) minWhiteRatio, /**< Ratio of pixels that must be white in a candidate region. */
    (int)(16) blockSizeX, /**< Must be the same value as in the CNSRegionProvider. */
    (int)(16) blockSizeY, /**< Must be the same value as in the CNSRegionProvider. */
    (unsigned)(3) maxNumberOfRegions, /**< The maximum number of regions created. */
  }),
});

class PenaltyMarkRegionsProvider : public PenaltyMarkRegionsProviderBase
{
  /** A pixel region as part of a union find structure. */
  struct Region
  {
    Region* parent = this; /**< Parent region. If pointing to itself, this is the root. */
    unsigned short upper; /**< Upper border. Inclusive. */
    unsigned short lower; /**< Lower border. Exclusive. */
    unsigned short left; /**< Left border. Inclusive. */
    unsigned short right; /**< Right border. Inclusive. */
    unsigned short pixels; /**< The overall number of pixels in this region. */
    unsigned short whitePixels; /**< The overall number of white pixels in this region. */

    /**
     * Constructor.
     * @param upper The upper border of the region (inclusive).
     * @param lower The lower border of the region (exclusive).
     * @param x The pixel column of this region.
     * @param isWhite It this a white region?
     */
    Region(unsigned short upper, unsigned short lower, unsigned short x, bool isWhite)
      : upper(upper),
        lower(lower),
        left(x),
        right(x + 1),
        pixels(lower - upper),
        whitePixels(isWhite ? pixels : 0) {}

    /**
     * Find the root of the current region (with path compression).
     * @return The root of the current region.
     */
    Region* getRoot()
    {
      Region* r = this;
      while(r != r->parent)
      {
        Region* r2 = r;
        r = r->parent;
        r2->parent = r->parent;
      }
      return r;
    }
  };

  std::vector<Region> regions; /**< The regions of non-green pixels (left to right, bottom to bottom). */
  std::vector<unsigned short> extendedLower; /**< A table that maps y coordinates to y coordinates with region extension. */
  std::vector<Boundaryi> cnsRegions; /**< The CNS regions that will be provided. */
  std::vector<Vector2i> spots; /**< The spots that will be provided. */

  void update(PenaltyMarkRegions& thePenaltyMarkRegions) override;
  void update(CNSPenaltyMarkRegions& theCNSPenaltyMarkRegions) override {theCNSPenaltyMarkRegions.regions = cnsRegions;}
  void update(PenaltyMarkSpots& thePenaltyMarkSpots) override {thePenaltyMarkSpots.spots = spots;}

  /**
   * Initializes the extendedLower table.
   * @param upperBound The smallest y coordinate that can be expected.
   */
  void initTables(unsigned short upperBound);

  /**
   * Initializes the scan lines containing the non-green regions.
   * @param upperBound The smallest y coordinate that can be expected.
   * @return Could the regions be initialized? This method returns false
   *         if there would be too many regions. In that case, the image
   *         is probably very noisy.
   */
  bool initRegions(unsigned short upperBound);

  /**
   * Merges all neighboring regions.
   * @param xStep The distance between neighboring low-res scan lines.
   */
  void unionFind(int xStep);

  /**
   * Analyses the merged regions finding candidates for penalty marks.
   * This method also updates the cnsRegion member.
   * @param upperBound The smallest y coordinate that can be expected.
   * @param xStep The distance between neighboring low-res scan lines.
   * @param searchRegions The regions that should be searched for the center of a penalty mark.
   */
  void analyseRegions(unsigned short upperBound, int xStep, std::vector<Boundaryi>& searchRegions);

public:
  PenaltyMarkRegionsProvider() {regions.reserve(1000);}
};
