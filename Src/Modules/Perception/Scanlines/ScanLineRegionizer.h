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
#include "ImageProcessing/PixelTypes.h"
#include "Framework/Module.h"

#include <limits>

MODULE(ScanLineRegionizer,
{,
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
    (Vector2f)(1500.f, 0.f) additionalSmoothingPoint,   /**< On field distance in mm up to which additional smoothing is applied */
    (float)(13) edgeThreshold,                          /**< The edge threshold. */
    (unsigned char)(20) luminanceSimilarityThreshold,   /**< Maximum luminance difference for two regions to become united */
    (unsigned char)(20) hueSimilarityThreshold,         /**< Maximum hue difference for two regions to become united */
    (unsigned char)(26) saturationSimilarityThreshold,  /**< Maximum saturation difference for two regions to become united */
    (unsigned char)(138) initialMinSaturation,          /**< Standard minimum field saturation for field classification */
    (int)(520) lowerMinRegionSize,                      /**< Minimal size in covered scanline pixels for initial field regions on the lower camera */
    (int)(640) upperMinRegionSize,                      /**< Minimal size in covered scanline pixels for initial field regions on the upper camera */
    (float)(0.9f) baseLuminanceReduction,               /**< Rather underestimate the baseLuminance as it is used for noise filtering */
    (short)(2) verticalGridScanMinStep,                 /**< Minimum distance between two vertical scan steps */
    (bool)(true) prelabelAsWhite,                       /**< Label upper regions that are brighter than both neighbors as white during region building */
    (short)(20) maxPrelabelRegionSize,                  /**< Maximum region size to prelabel as white */
    (short)(12) maxRegionSizeForStitching,              /**< Maximum size in pixels of a none region between field and white or field and field for stitching */
    (int)(400) estimatedFieldColorInvalidationTime,     /**< Time in ms until the EstimatedFieldColor is invalidated */
      }),
});

class ScanLineRegionizer : public ScanLineRegionizerBase
{
  struct EstimatedFieldColor
  {
    // no support for circular hue range here, field won't be on the zero crossing
    unsigned char minHue; /**< Minimal field hue */
    unsigned char maxHue; /**< Maximum field hue */
    unsigned char minSaturation; /**< Minimal field saturation */
    unsigned char maxLuminance; /**< Maximum field luminance */
    unsigned int lastSet; /**< Timestamp of when the estimated field colors were last set */
  };

  /**
   * Forest-like (graph) data structure for union find. The InternalRegion is a node in the graph.
   * Each node knows only its parent node, which allows to quickly unite trees and find the topmost node for every tree node.
   */
  struct InternalRegion : ScanLineRegion
  {
    InternalRegion(unsigned short from, unsigned short to, PixelTypes::GrayscaledPixel y, PixelTypes::HuePixel h, PixelTypes::GrayscaledPixel s) :
      ScanLineRegion(from, to, ScanLineRegion::unset), // initialize as unset for optimized classification into field/none/white in later steps
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
    unsigned short rank = 0; /**< Rank for union-find (maximal tree-depth). Regions with higher rank become parent regions to keep tree depth low. */

  public:
    /**
     * Finds the representative region for this set and does path compression.
     * This InternalRegions parent node will be set to the tree root.
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
     * Unites this region set with the other region set.
     * The representative values for the new parent region will be updated to represent the union-region.
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
   * Contains info about a scan-line scan run, that are needed in all sub-functions of the scan.
   */
  template <int filterSize>
  struct ScanRun
  {
    const bool horizontal;               /**< true for horizontal scan */
    const int scanLinePosition;          /**< height or x-position of the scan-line */
    const unsigned int scanStart;        /**< left- or bottommost pixel of the scan-line */
    const unsigned int scanStop;         /**< right- or topmost pixel of the scan-line */
    unsigned int leftScanEdgePosition{}; /**< Left or lower edge of the current region */
    unsigned int& lowerScanEdgePosition = leftScanEdgePosition; /**< name alias for vertical scan */
    int (*gaussV)(const PixelTypes::GrayscaledPixel*, const unsigned int){}; /**< 1D vertical gaussV smoothing filter that works on the image */
    int (*gaussH)(const PixelTypes::GrayscaledPixel*){};                    /**< 1D horizontal gaussV smoothing filter that works on the image */
    int (*gaussSecond)(::std::array<int, filterSize>&, int);                /**< 1D gauss smoothing filter that works on an array */
    int (*gradient)(::std::array<int, filterSize>&, int);                   /**< 1D sobel gradient filter */
    std::array<int, filterSize> leftGaussBuffer;  /**< buffer for the left or lower grid point and for sobel scans*/
    std::array<int, filterSize>& lowerGaussBuffer = leftGaussBuffer; /**< name alias for vertical scan */
    std::array<int, filterSize> rightGaussBuffer; /**< buffer for the right or upper grid point, centered around grid point -> gridX is at index 1 */
    std::array<int, filterSize>& upperGaussBuffer = rightGaussBuffer; /**< name alias for vertical scan */

    ScanRun(bool horizontal, int scanLinePosition, unsigned int scanStart, unsigned int scanStop):
      horizontal(horizontal),
      scanLinePosition(scanLinePosition),
      scanStart(scanStart),
      scanStop(scanStop)
    { };

  public:
    [[nodiscard]] bool vertical() const
    {
      return !horizontal;
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
   * Uses a 3x3 pixel range centered around the checked points when comparing samples.
   * The scanning is done in two stages:
   * <ol>
   * <li>Grid scan. Samples from the scan line are drawn at the crossing points with the vertical scan-lines from the scan grid.</li>
   * <li>If two neighboring grid points luminances differ significantly,
   *     the part in between will be scanned to find the optimal split point between to scan-line regions.</li>
   * </ol>
   * <p>
   * Also small regions that are significantly lighter than both neighboring regions may already become labeled as white.
   * This helps with identifying far away field lines, as otherwise they often are falsely labeled as field.
   * @param y The height of the scan line in the image.
   * @param regions The regions to be filled.
   * @param leftmostX Left-side starting point of the scan-line
   * @param rightmostX Right-side end point of the scan-line
   */
  void scanHorizontal(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const;

  /**
   * Creates regions along a horizontal line.
   * Uses a 5x5 pixel range centered around the checked points when comparing samples.
   * The scanning is done in two stages:
   * <ol>
   * <li>Grid scan. Samples from the scan line are drawn at the crossing points with the vertical scan-lines from the scan grid.</li>
   * <li>If two neighboring grid points luminances differ significantly,
   *     the part in between will be scanned to find the optimal split point between to scan-line regions.</li>
   * </ol>
   * <p>
   * Also small regions that are significantly lighter than both neighboring regions may already become labeled as white.
   * This helps with identifying far away field lines, as otherwise they often are falsely labeled as field.
   * @param y The height of the scan line in the image.
   * @param regions The regions to be filled.
   * @param leftmostX Left-side starting point of the scan-line
   * @param rightmostX Right-side end point of the scan-line
   */
  void scanHorizontalAdditionalSmoothing(unsigned int y, std::vector<InternalRegion>& regions, const unsigned int leftmostX, const unsigned int rightmostX) const;

  /**
   * Creates regions along a vertical scan line.
   * @param line The scan grid line on which the scan is performed.
   * @param middle The y coordinate at which to switch between 5x5 and 3x3 filter
   * @param top The y coordinate (inclusive) below which the useful part of the image is located.
   * @param regions he regions to be filled.
   */
  void scanVertical(const ScanGrid::Line& line, int middle, int top, std::vector<InternalRegion>& regions) const;

  /**
   * Find an exact edge position in the scanRun in the subsegment designated by startPos and stopPos.
   * The edge position is at the pixel with the highest or lowest value (depending on the value of maxEdge)
   * of a Sobel filter applied to it.
   * <p>
   * A region will be added to the regions vector according to the scanRun info and the found edge position.
   * The found edge position is set as new leftEdgePosition in the scanRun.
   * @tparam filterSize size in pixels of the gauss and sobel filter kernel
   * @param regions regions vector to add the new scan-line region to
   * @param scanRun info about the scan run
   * @param startPos start position of the scan
   * @param stopPos stop position of the scan
   * @param maxEdge whether to search for black-to-white edge or a white-to-black edge
   */
  template <int filterSize>
  void findEdgeInSubLineHorizontal(std::vector<InternalRegion>& regions, ScanRun<filterSize>& scanRun, unsigned int startPos, unsigned int stopPos, bool maxEdge) const;

  /**
   * Find an exact edge position in the scanRun in the subsegment designated by startPos and stopPos.
   * The edge position is at the pixel with the highest or lowest value (depending on the value of maxEdge)
   * of a Sobel filter applied to it.
   * <p>
   * A region will be added to the regions vector according to the scanRun info and the found edge position.
   * The found edge position is set as new lowerScanEdgePosition in the scanRun.
   * @tparam filterSize size in pixels of the gauss and sobel filter kernel
   * @param regions regions vector to add the new scan-line region to
   * @param scanRun info about the scan run
   * @param startPos start position of the scan
   * @param stopPos stop position of the scan
   * @param maxEdge whether to search for black-to-white edge or a white-to-black edge
   */
  template <int filterSize>
  void findEdgeInSubLineVertical(std::vector<InternalRegion>& regions, ScanRun<filterSize>& scanRun, unsigned int startPos, unsigned int stopPos, bool maxEdge) const;

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
   * Excludes regions already labeled as white to avoid them being turned into field.
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

  /**
   * Classify all yet unclassified horizontal regions as field or not.
   * Classification based on estimated field color.
   * @param y the in image heights of the scan lines
   * @param regions the regions
   */
  void classifyFieldHorizontal(const std::vector<unsigned short>& y, std::vector<std::vector<InternalRegion>>& regions) const;


  /**
   * Classify all yet unclassified vertical regions as field or not.
   * Classification based on estimated field color.
   * @param regions the regions
   */
  void classifyFieldVertical(std::vector<std::vector<InternalRegion>>& regions) const;

  /**
   * Classify a single yet unclassified region as field or not.
   * Classification based on estimated field color.
   * @param region the region
   * @param regionHeight the height of the region in image coordinates (overwritten in vertical case)
   * @param horizontal Are the regions on horizontal (true) or vertical (false) scan lines
   */
  void classifyFieldSingleRegion(InternalRegion& region, float regionHeight, bool horizontal) const;

  /**
   * Saturation of very dark field pixels is often 0. Also, further away field regions can have lower saturation.
   * Adjusts saturation threshold from estimated field color accordingly.
   * @param region the region
   * @param regionHeight the height of the region in image coordinates
   * @return adjusted saturation threshold
   */
  unsigned char fieldClassificationSaturationThreshold(const InternalRegion& region, float regionHeight) const;

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
   * Emplace computed scan-line regions into the ColorScanLineRegionsHorizontal representation.
   * Neighboring regions of the same classification will be merged into one region.
   * The representation has to be cleared beforehand or the new regions will be appended.
   * @param colorScanLineRegionsHorizontal the representation to emplace into
   * @param yPerScanLine heights of the scan-lines in the camera image
   * @param regionsPerScanLine a vector of scan-line regions per scan-line
   */
  static void emplaceInScanLineRegionsHorizontal(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal,
                                          const std::vector<unsigned short>& yPerScanLine,
                                          const std::vector<std::vector<InternalRegion>>& regionsPerScanLine);

  /**
   * Emplace computed scan-line regions into the ColorScanLineRegionsVerticalClipped representation.
   * Neighboring regions of the same classification will be merged into one region.
   * The representation has to be cleared beforehand or the new regions will be appended.
   * @param colorScanLineRegionsVertical the representation to emplace into
   * @param xPerScanLine x-Position of the scan-lines in the camera image
   * @param regionsPerScanLine a vector of scan-line regions per scan-line
   */
  static void emplaceInScanLineRegionsVertical(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped,
                                                 const std::vector<unsigned short>& xPerScanLine,
                                                 const std::vector<std::vector<InternalRegion>>& regionsPerScanLine);

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

  /**
   * Debug drawing that shows the estimated field color range, either for the pass on the horizontal or the vertical scan lines.
   */
  void drawEstimatedFieldColorRangeHorizontal(int dataPoints) const;

  /**
   * Debug drawing that shows the estimated field color range, either for the pass on the horizontal or the vertical scan lines.
   */
  void drawEstimatedFieldColorRangeVertical(int dataPoints) const;


public:
  /**
   * Computes an average over hue values that correctly handles the circular hue range.
   * The old average and the additional value are weighted (dataPoints - 1) : 1.
   * @param hueValue Current hue average.
   * @param hueAddition New data point, that shall be incorporated in the average.
   * @param dataPoints Number of data points including the new one.
   * @return New average value in range [0,256)
   */
  static float hueAverage(float hueValue, float hueAddition, int dataPoints);

private:
  PixelTypes::GrayscaledPixel baseLuminance; /**< heuristically approximated average luminance of the image.
  * Used as a min luminance threshold for filtering out irrelevant edges and noise */
  PixelTypes::GrayscaledPixel baseSaturation; /**< heuristically approximated average saturation of the image.
  * Used as a min luminance threshold for filtering out irrelevant edges and noise */
  EstimatedFieldColor estimatedFieldColor; /**< Field color range estimated for the current image */
};
