//
//  TImageBinarizer.hpp
//  B-Human
//
//  Created by Peter Schulz on 12/06/16.
//
//

#pragma once

#include "TImage.h"
#include "PixelTypes.h"
#include <vector>

class TImageBinarizer
{
public:
  struct Interval
  {
    //! An interval consists of all points (x,y) with xLo<=x && x<=xHi
    int xLo, xHi, y;
    
    //! index of the connected component of black regions to which this interval belongs
    int region;
    
    //! auxiliary pointer for union-find.
    /*! Pointer to an earlier (when scanning rowwise) interval in
     the same region. If it points to itself, this is the first interval.
     */
    Interval* parent;
    
    Interval () :xLo(0), xHi(0), y(0), region(0), parent(NULL) {};
    Interval (int xLo, int xHi, int y) :xLo(xLo), xHi(xHi), y(y), region(0), parent(NULL) { }
  };
  
  struct IntervalSet
  {
    std::vector<Interval> rle;
    
    //! Groups connected intervals.
    void groupIntervals();
  protected:
    //! Auxiliary routine for groupRegions
    /*! Returns, whether run and flw touch and run is one line below follow. */
    bool touch (Interval* run, Interval* flw);
    
    //! Auxiliary routine for groupRegions
    /*! Returns, whether run is one line below flw and horizontally ahead
     or more than one line below flw. */
    bool ahead (Interval* run, Interval* flw);
    
    //! Auxiliary routine for unite.
    /*! Finds the root of \c iv. This is the first interval of the
     connected component. It modifies all intervals along the way to
     point directly to the final parent.
     */
    void pathCompress (Interval* iv);
    
    //! Auxiliary routine for group regions.
    /*! Unites the connected components of iv1 and iv2.
     */
    void unite (Interval* iv1, Interval* iv2);
    
    //! Auxiliary routine for groupRegion
    /*! Initializes the .parent pointer of all intervals to point to itself,
     declaring every interval as a single region.
     */
    void initialize ();
    
    //! Auxiliary routine for groupRegion
    /*! Assumes that the regions are defined by the .parent pointer
     (All interval having the same root are one region) and gives
     the regions (in .region) consecutives numbers starting with 0.
     */
    void setRegionIndex ();
  };
  
  struct Region
  {
    int integralX, integralY;
    
    static void extractRegions(IntervalSet& intervalSet, std::vector<Region>& regions);
    
    Region() : integralX(0), integralY(0) {}
  };
  
  TImageBinarizer();
  
  //! Binarizes the specified (sub-)image.
  /*!
   \param image the image
   \param xOffset sub-image x offset
   \param xLength sub-image width
   \param yOffset sub-image y offset
   \param yLength sub-image height
   */
  IntervalSet binarize(TImage<PixelTypes::GrayscaledPixel> image, int xOffset, int xLength, int yOffset, int yLength);
  
  PixelTypes::GrayscaledPixel threshold;
  int minLength;
};