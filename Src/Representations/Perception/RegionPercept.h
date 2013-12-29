/**
* @file Representations/Perception/RegionPercept.h
*
* Representention of the seen regions
*
* @author jeff
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Platform/BHAssert.h"
#include <vector>

#define MAX_SEGMENTS_COUNT 2000
#define MAX_REGIONS_COUNT 2000

#define GAUSS_SUM(x) ( x * ( x + 1 ) / 2 )
#define GAUSS_SUM2(x) ( x * ( x + 1 ) * ( 2 * x + 1 ) / 6 )

/**
 * Return a ColorRGBA value in which a region/segment
 * with the color "color" should be drawn on the field
 * @param color the color of the object to be drawn
 */
inline ColorRGBA getOnFieldDrawColor(ColorClasses::Color color)
{
  switch(color)
  {
  case ColorClasses::black:
    return ColorRGBA(ColorClasses::white);
    break;
  case ColorClasses::white:
    return ColorRGBA(ColorClasses::black);
    break;
  case ColorClasses::none:
    return  ColorRGBA(255, 0, 0);
    break;
  case ColorClasses::yellow:
    return ColorRGBA(ColorClasses::blue);
    break;
  case ColorClasses::blue:
    return ColorRGBA(ColorClasses::yellow);
    break;
  case ColorClasses::red:
    return ColorRGBA(ColorClasses::blue);
    break;
  default:
    return ColorRGBA(color);
  }
}

/**
 * @class RegionPercept
 *
 * The RegionPercept from the segmentation.
 */
class RegionPercept: public Streamable
{
private:
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  }

public:
  class Segment;

  /**
   * @class Region
   * A class to store a region
   */
  class Region
  {
  public:
    /**
     *Default Constructor
     */
    Region()
    {
      children.reserve(64);
    }

    /**
     * If the region was merged with another one, this
     * function will return the region it was merged to.
     * @return the root region
     * */
    Region* getRootRegion() const;

    /** Calculate the center of the region in the image.
     * @return the center of the region in the image
     */
    Vector2<int> getCenter() const;

    /** Calculate the (p=)0th (q=)0th moment
     * @return the 0th0th moment
     * */
    int calcMoment00() const;

    /** Calculate the (p=)1th (q=)0th moment
     * @return the 1th0th moment
     * */
    int calcMoment10() const;

    /** Calculate the (p=)0th (q=)1th moment
     * @return the 0th1th moment
     * */
    int calcMoment01() const;

    /** Calculate the (p=)1th (q=)1th centralized moment
     * @param swp_x x-Coordinate of the "Schwerpunkt"
     * @param swp_y y-Coordinate of the "Schwerpunkt"
     * @return the 1th1th moment
     * */
    float calcCMoment11(int swp_x, int swp_y) const;

    /** Calculate the (p=)2th (q=)0th centralized moment
     * @param swp_x x-Coordinate of the "Schwerpunkt"
     * @return the 2th0th moment
     * */
    int calcCMoment20(int swp_x) const;

    /** Calculate the (p=)0th (q=)2th centralized moment
     * @param swp_y y-Coordinate of the "Schwerpunkt"
     * @return the 0th2th moment
     * */
    float calcCMoment02(int swp_y) const;

    /**
     * Merges another region to this one. This will be the parent (root)
     * region, the other one will be "dead".
     * @param other region to merge into this one
     */
    void mergeWithRegion(Region* other);

    std::vector<Segment*> children; /**< The child segments of this region. */
    std::vector<Region*> neighborRegions; /**< The neighbor Regions of this region. */
    int min_y, /**< The minimum y value of the childs. */
        max_y; /**< The maximum y value of the childs. */
    Region* root; /**< The root region, if this region was merged to another one. */
    ColorClasses::Color color; /**< The color of this region. */
    int size; /**< The size in (explored) pixels of the this region. */
  };

  /**
   * @class Segment
   *
   * A class to store a Segment
   */
  class Segment
  {
  public:
    /** Default constructor */
    Segment()
    {
      region = NULL;
      link = NULL;
    }

    /** The < operator
     * @param s2 Segment to compare to
     * @return is this < s2?
     */
    bool operator<(Segment* s2);

    int x, /**< The x coordinate of the segment. */
        y, /**< The y coordinate where the segment start. */
        length; /**< The length (in y direction) of the segment. */
    Segment* link; /**< Link to the previous segment in the region. */
    int explored_min_y; /**< The minimum explored y coordiante. */
    int explored_max_y; /**< The maximum explored y coordinate. */
    int explored_size; /**< Aproximated size of this segment. */
    Region* region; /**< The region this segment belongs to. */
    ColorClasses::Color color; /**< The color of this segment. */
  };

  Segment segments[MAX_SEGMENTS_COUNT]; /**< This array stores all the segments. */
  Region regions[MAX_REGIONS_COUNT]; /**< This array stores all the regions. */
  int segmentsCounter, /**< A counter for the segments. This counter always points to index after
                            the last segment in segments. */
      regionsCounter;  /**< A counter the the regions. This counter always points to the index after
                            the last region in regions. */
  int gridStepSize; /**< The number of pixels between the scanlines. */

  /** Constructor */
  RegionPercept()
  {
    segmentsCounter = 0;
    regionsCounter = 0;
  }

  /**
  * The method draws the percept.
  */
  void draw() const;
};
