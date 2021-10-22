/**
 * @file ScanLineRegionizer.h
 *
 * This file declares a module that segments the image horizontally and
 * vertically by detecting edges and applying heuristics to classify
 * the regions between them.
 *
 * @author Lukas Malte Monnerjahn
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/RelativeFieldColorsParameters.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/RelativeFieldColors.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Module/Module.h"

#include <limits>

MODULE(ScanLineRegionizer,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(FrameInfo),
  REQUIRES(RelativeFieldColors),
  REQUIRES(RelativeFieldColorsParameters),
  REQUIRES(ScanGrid),
  PROVIDES(ColorScanLineRegionsHorizontal),
  PROVIDES(ColorScanLineRegionsVerticalClipped),
  DEFINES_PARAMETERS(
  {,
    (Vector2f)(Vector2f(1500.f, 0.f)) additionalSmoothingPoint, /**< On field distance in mm up to which additional smoothing is applied */
    (float)(13) edgeThreshold,                         /**< The edge threshold. */
    (unsigned short)(12) minHorizontalScanLineDistance, /**< Minimal distance between horizontal scan lines in px */
    (unsigned char)(20) luminanceSimilarityThreshold,  /**< Maximum luminance difference for two regions to become united */
    (unsigned char)(20) hueSimilarityThreshold,        /**< Maximum hue difference for two regions to become united */
    (unsigned char)(26) saturationSimilarityThreshold, /**< Maximum saturation difference for two regions to become united */
    (int)(520) lowerMinRegionSize,                     /**< Minimal size in covered scanline pixels for initial field regions on the lower camera */
    (int)(640) upperMinRegionSize,                     /**< Minimal size in covered scanline pixels for initial field regions on the upper camera */
    (float)(0.9f) baseLuminanceReduction,              /**< Rather underestimate the baseLuminance as it is used for noise filtering */
    (short)(2) verticalGridScanMinStep,                /**< Minimum distance between two vertical scan steps */
    (bool)(true) prelabelAsWhite,                      /**< Label upper regions that are brighter than both neighbors as white during region building */
    (short)(20) maxPrelabelRegionSize,                 /**< Maximum region size to prelabel as white */
    (short)(12) maxRegionSizeForStitching,             /**< Maximum size in pixels of a none region between field and white or field and field for stitching */
    (int)(400) estimatedFieldColorInvalidationTime,    /**< Time in ms until the EstimatedFieldColor is invalidated */
      }),
});

class ScanLineRegionizer : public ScanLineRegionizerBase
{
  struct EstimatedFieldColor
  {
    // no support for circular hue range here, field wont be on the zero crossing
    unsigned char minHue; /**< Minimal field hue */
    unsigned char maxHue; /**< Maximum field hue */
    unsigned char minSaturation; /**< Minimal field saturation */
    unsigned char maxLuminance; /**< Maximum field luminance */
    unsigned int lastSet; /**< Timestamp of when the estimated field colors were last set */
  };

  struct InternalRegion : ScanLineRegion
  {
    InternalRegion(unsigned short from, unsigned short to, PixelTypes::GrayscaledPixel y, PixelTypes::HuePixel h, PixelTypes::GrayscaledPixel s) :
      ScanLineRegion(from, to, PixelTypes::Color::black), // initialize as black for optimized classification into field/none/white in later steps
      y(y),
      h(h),
      s(s),
      parent(nullptr)
    {
      ASSERT(to > from);
      regionSize = to - from;
    }

    PixelTypes::GrayscaledPixel y; /**< A representative Y value of this region. */
    PixelTypes::HuePixel h;        /**< A representative H value of this region. */
    PixelTypes::GrayscaledPixel s; /**< A representative S value of this region. */
    int regionSize;                /**< Accumulated size over this and all child regions */

  private:
    InternalRegion* parent;  /**< Parent for union-find. */
    unsigned short rank = 0; /**< Rank for union-find. */

  public:
    /**
     * Finds the representative region for this set and does path compression.
     * @return This sets representative region.
     */
    InternalRegion* findSet()
    {
      if(parent == nullptr)
        return this;
      parent = parent->findSet();
      return parent;
    }

    /**
     * Unites this region set this the other region set.
     * @param other The other region to unite with.
     */
    void unite(InternalRegion& other)
    {
      InternalRegion* a = findSet();
      InternalRegion* b = other.findSet();
      if(a == b)
        return;
      if(a->rank > b->rank)
      {
        b->parent = a;
        a->updateValues(b);
      }
      else
      {
        a->parent = b;
        b->updateValues(a);
        if(a->rank == b->rank)
          ++b->rank;
      }
    }

  private:
    /**
     * Updates values of a new parent region that represents the whole set.
     * @param other New values to incorporate.
     */
    void updateValues(const InternalRegion* const other)
    {
      const int regionSum = regionSize + other->regionSize;
      y = static_cast<PixelTypes::GrayscaledPixel>((regionSize * y + other->regionSize * other->y) / regionSum);
      s = static_cast<PixelTypes::GrayscaledPixel>((regionSize * s + other->regionSize * other->s) / regionSum);
      // special handling for circular hue range
      if(static_cast<int>(h) - static_cast<int>(other->h) > std::numeric_limits<signed char>::max()) // == std::numeric_limits<unsigned char>::max() / 2
        h += static_cast<unsigned char>((static_cast<int>(other->h) - static_cast<int>(h) + std::numeric_limits<unsigned char>::max()) * other->regionSize / regionSum);
      else if(static_cast<int>(other->h) - static_cast<int>(h) > std::numeric_limits<signed char>::max())
        h -= static_cast<unsigned char>((static_cast<int>(h) - static_cast<int>(other->h) + std::numeric_limits<unsigned char>::max()) * other->regionSize / regionSum);
      else if((static_cast<int>(other->h) - static_cast<int>(h) >= 0))
        h += static_cast<unsigned char>((static_cast<int>(other->h) - static_cast<int>(h)) * other->regionSize / regionSum);
      else
        h -= static_cast<unsigned char>((static_cast<int>(h) - static_cast<int>(other->h)) * other->regionSize / regionSum);
      regionSize = regionSum;
    }
  };

  /**
   * Updates the horizontal color scan line regions.
   * @param colorScanLineRegionsHorizontal The provided representation.
   */
  void update(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal) override;

  /**
   * Updates the vertical color scan line regions.
   * @param colorScanLineRegionsVertical The provided representation.
   */
  void update(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped) override;

  /**
   * Creates regions along a horizontal line.
   * @param y The height of the scan line in the image.
   * @param regions The regions to be filled.
   */
  void scanHorizontalGrid(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const;

  /**
   * Creates regions along a horizontal line.
   * @param y The height of the scan line in the image.
   * @param regions The regions to be filled.
   */
  void scanHorizontal(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const;

  /**
   * Creates regions along a horizontal line.
   * @param y The height of the scan line in the image.
   * @param regions The regions to be filled.
   */
  void scanHorizontalGridAdditionalSmoothing(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const;

  /**
   * Creates regions along a horizontal line.
   * @param y The height of the scan line in the image.
   * @param regions The regions to be filled.
   */
  void scanHorizontalAdditionalSmoothing(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const;

  /**
   * Determines the horizontal scan start, excluding areas outside the field boundary
   * @param y The height of the scan line in the image.
   * @return The x-coordinate of where to start the scan
   */
  [[nodiscard]] int horizontalScanStart(int x, int y) const;

  /**
   * Determines the horizontal scan stop, excluding areas outside the field boundary
   * @param y The height of the scan line in the image.
   * @return The x-coordinate of where to stop the scan
   */
  [[nodiscard]] int horizontalScanStop(int x, int y) const;

  /**
   * Creates regions along a vertical scan line.
   * @param line The scan grid line on which the scan is performed.
   * @param middle The y coordinate at which to switch between 5x5 and 3x3 filter
   * @param top The y coordinate (inclusive) below which the useful part of the image is located.
   * @param regions he regions to be filled.
   */
  void scanVerticalGrid(const ScanGrid::Line& line, int middle, int top, std::vector<InternalRegion>& regions) const;

  /**
   * Creates regions along a vertical scan line.
   * @param line The scan grid line on which the scan is performed.
   * @param middle The y coordinate at which to switch between 5x5 and 3x3 filter
   * @param top The y coordinate (inclusive) below which the useful part of the image is located.
   * @param regions he regions to be filled.
   */
  void scanVertical(const ScanGrid::Line& line, int middle, int top, std::vector<InternalRegion>& regions) const;

  /**
   * Unites similar horizontal scan line regions.
   * @param x The x coordinates of the regions.
   * @param regions The regions (grouped by scan line), scan lines sorted from bottom to top,
   * regions in the scan lines sorted ascending by pixel number from left to right.
   */
  void uniteHorizontalFieldRegions(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions) const;

  /**
   * Unite similar vertical scan line regions.
   * @param x The x coordinates of the regions.
   * @param regions The regions (grouped by scan line), scan lines sorted from left to right,
   * regions in the scan lines sorted descending by pixel number from bottom to top.
   */
  void uniteVerticalFieldRegions(const std::vector<unsigned short>& x, std::vector<std::vector<InternalRegion>>& regions) const;

  /**
   * Checks whether two regions are similar enough to unite them.
   * @param a One region.
   * @param b The other region.
   * @return true, if the regions are similar enough to be united
   */
  bool areSimilar(InternalRegion& a, InternalRegion& b) const;

  /**
   * Classifies united regions into field and not field.
   * @param xy x or y positions of the scan lines.
   * @param regions The regions to classify.
   * @param horizontal Are the regions on horizontal (true) or vertical (false) scan lines
   */
  void classifyFieldRegions(const std::vector<unsigned short>& xy, std::vector<std::vector<InternalRegion>>& regions, bool horizontal);

  void classifyFieldHorizontal(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions) const;

  void classifyFieldVertical(std::vector<std::vector<InternalRegion>>& regions) const;

  /**
   * Decides whether a region qualifies as field.
   * @param region The region
   * @return true, if the region is field
   */
  [[nodiscard]] bool regionIsField(const InternalRegion* const region) const;

  /**
   * Classifies some regions as white.
   * @param regions The regions (grouped by scan line).
   */
  void classifyWhiteRegionsWithThreshold(std::vector<std::vector<InternalRegion>>& regions) const;

  /**
   * Checks if the region fulfills basic characteristics for being prelabeled as white
   * @param checkedRegion The tested region
   * @return True, if the region fulfills basic characteristics for being prelabeled as white
   */
  [[nodiscard]] bool prelabelWhiteCheck(const InternalRegion& checkedRegion) const;

  /**
   * Checks if the region neighbor allows the checked region to be prelabeled as white
   * @param checkedRegion The tested region
   * @param neighborRegion The neighbor region
   * @return True, if the region neighbor allows the checked region to be prelabeled as white
   */
  [[nodiscard]] bool prelabelWhiteNeighborCheck(const InternalRegion& checkedRegion, const InternalRegion& neighborRegion) const;

  /**
   * Replaces small "none" regions in between "field" and "white" or "field" and "field".
   * @param regions The regions.
   * @param horizontal Whether the stitching is done on horizontal or vertical scan line regions.
   */
  void stitchUpHoles(std::vector<std::vector<InternalRegion>>& regions, bool horizontal) const;

  /**
   * Checks by timestamp if the EstimatedFieldColor is still presumed valid.
   * @return True, if the EstimatedFieldColor is valid.
   */
  [[nodiscard]] bool isEstimatedFieldColorValid() const;

  /**
   * Gets an image grayscale value that is representative for a given horizontal region.
   * @param image The image.
   * @param from The leftmost x coordinate of the region (inclusive).
   * @param to The rightmost x coordinate of the region (exclusive).
   * @param y The y coordinate of the region.
   */
  static PixelTypes::GrayscaledPixel getHorizontalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image, unsigned int from, unsigned int to, unsigned int y);

  /**
   * Gets an image hue value that is representative for a given horizontal region.
   * @param image The image.
   * @param from The leftmost x coordinate of the region (inclusive).
   * @param to The rightmost x coordinate of the region (exclusive).
   * @param y The y coordinate of the region.
   */
  static PixelTypes::HuePixel getHorizontalRepresentativeHueValue(const Image<PixelTypes::HuePixel>& image, unsigned int from, unsigned int to, unsigned int y);

  /**
   * Gets an image grayscale value that is representative for a given vertical region.
   * @param image The image.
   * @param x The x coordinate of the region.
   * @param from The topmost y coordinate of the region (inclusive).
   * @param to The bottommost y coordinate of the region (exclusive).
   */
  static PixelTypes::GrayscaledPixel getVerticalRepresentativeValue(const Image<PixelTypes::GrayscaledPixel>& image, unsigned int x, unsigned int from, unsigned int to);

  /**
   * Gets an image hue value that is representative for a given vertical region.
   * @param image The image.
   * @param x The x coordinate of the region.
   * @param from The topmost y coordinate of the region (inclusive).
   * @param to The bottommost y coordinate of the region (exclusive).
   */
  static PixelTypes::HuePixel getVerticalRepresentativeHueValue(const Image<PixelTypes::HuePixel>& image, unsigned int x, unsigned int from, unsigned int to);

  /**
   * Approximates the images average luminance.
   */
  void approximateBaseLuminance();

  /**
   * Approximates the images average saturation.
   */
  void approximateBaseSaturation();

public:
  /**
   * Computes an average over hue values that correctly handles the circular hue range.
   * The old average and the additional value are weighted (dataPoints - 1) : 1.
   * @param hueValue Current hue average.
   * @param hueAddition New data point, that shall be incorporated in the average.
   * @param dataPoints Number of data points including the new one.
   * @return New average value.
   */
  static float hueAverage(float hueValue, float hueAddition, int dataPoints);

private:
  PixelTypes::GrayscaledPixel baseLuminance; /**< heuristically approximated average luminance of the image.
  * Used as a min luminance threshold for filtering out irrelevant edges and noise */
  PixelTypes::GrayscaledPixel baseSaturation; /**< heuristically approximated average saturation of the image.
  * Used as a min luminance threshold for filtering out irrelevant edges and noise */
  EstimatedFieldColor estimatedFieldColor; /**< Field color range estimated for the current image */
};
