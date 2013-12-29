/**
* @file Regionizer.h
* @author jeff
* @author benny
* @author Florian Maaß
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/ObstacleSpots.h"
#include "PointExplorer.h"

MODULE(Regionizer)
  REQUIRES(BodyContour)
  REQUIRES(Image)
  REQUIRES(ColorReference)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraMatrix)
  REQUIRES(FieldBoundary)
  REQUIRES(ObstacleSpots)
  REQUIRES(CameraInfo)
  PROVIDES_WITH_MODIFY_AND_DRAW(RegionPercept)
  LOADS_PARAMETER(int, gridStepSize) /**< The distance in pixels between neighboring scan lines. */
  LOADS_PARAMETER(int, skipOffset) /**< The maximum number of pixels to skip when grouping pixels to segments. */
  LOADS_PARAMETER(int[ColorClasses::numOfColors], minSegSize) /**< The minimal size/length for a segment in pixels for each color. */
  LOADS_PARAMETER(float[ColorClasses::numOfColors], regionLengthFactor) /**< The maximal allowed factor one segment is to be longer than another when grouping to a region */
  LOADS_PARAMETER(int, regionMaxSize) /**< The maximal number of segments per region */
  LOADS_PARAMETER(float, maxAngleDiff) /**< The maximal angle difference from one segment to another.
                                        The angle is the vector from the middle of the last segment to the next one. */
  LOADS_PARAMETER(int, exploreStepSize) /**< The distance in pixels between exploring scanlines */
END_MODULE

/**
 * @class Regionizer
 *
 * This class segments the image and create regions from the segments.
 */
class Regionizer: public RegionizerBase
{
private:
  typedef std::vector<Vector2<int> >::const_iterator CI;
  RegionPercept* regionPercept; /**< internal pointer to the RegionPercept */
  PointExplorer pointExplorer; /**< PointerExplorer instance for running in the image */

  /** Updates the RegionPercept */
  void update(RegionPercept& rPercept);

  /**
  * The method scans vertically for segments in the image.
  * It includes the scanning for goal / post segments
  */
  void scanVertically();
  /**
   * Unites the regions of two segments if certain criterions are met. If seg2
   * does not have a region yet, it will be added to seg1's region.
   * @param seg1 segment which already has region
   * @param seg2 segment(including it's region if it has one) which will be merge to seg1's region
   * @return whether the regions where united
   */
  bool uniteRegions(RegionPercept::Segment* seg1, RegionPercept::Segment* seg2);

  /**
   * Returns a pointer to a new segment (from the segments array within the RegionPercept).
   * @param x the x coordinate of the new segment
   * @param y the y coordinate of the new segment
   * @param length the length of the new segment
   * @param color the color of the new segment
   * @return pointer to the new segment or NULL if segments array is full
   */
  inline RegionPercept::Segment* addSegment(int x, int y, int length, ColorClasses::Color color);

  /**
   * Creates a new Region in the RegionPercept for the segment passed.
   * @param seg the segment to be added to the region
   * @return whether a region was created (the percept was not full)
   */
  inline bool createNewRegionForSegment(RegionPercept::Segment* seg);

  /**
   * Connects a segment to the already created regions. If the segment
   * does not touch a segment of the same color, a new region will be formed.
   * @param newSegment the segment to be connected
   * @param lastColumPointer a pointer to a segment in the last colum which is above newSegment
   * @param xDiff the difference in x coordinates from the last colum to the actual (=gridStepSize)
   * @return the lastColumpointer for the next segment
   */
  inline RegionPercept::Segment* connectToRegions(RegionPercept::Segment* newSegment, RegionPercept::Segment* lastColumPointer, int xDiff);

  /**
   * Builds the regions from the segments.
   */
  void buildRegions();
public:
  /**
  * Default constructor.
  */
  Regionizer();
};
