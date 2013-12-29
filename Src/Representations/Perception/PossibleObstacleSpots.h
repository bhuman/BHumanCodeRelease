/*
 * File:   LineSpots2.h
 * Author: arne
 *
 * Created on February 7, 2013, 9:41 AM
 */


#pragma once

#include "Tools/Streams/Streamable.h"
#include <vector>
#include "Representations/Infrastructure/Image.h"
class PossibleObstacleSpots : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    //FIXME
    STREAM_REGISTER_FINISH;
  }

public:

  /** This can be improved a lot :-) */
  struct Scanline
  {
    int xImg; /**< x position of this scanline in image coordinates */
    int spots[maxResolutionHeight]; /**< y coordinate of the spots from bottom to top */
    int spotCount;
    void clear()
    {
      spotCount = 0;
    }
  };

  Scanline scanlines[maxResolutionWidth]; /**< DO NOT USE scanlines.size(). use scanlineCount instead!! */
  int scanlineCount; /**< real count of scanlines */

  PossibleObstacleSpots()
  {
    scanlineCount = 0;
  }

  /**remove all spots*/
  void clear()
  {
    scanlineCount = 0;
  }

  /**Returns a reference to the next unused scanline*/
  Scanline& getNextScanline()
  {
    return scanlines[scanlineCount++];
  }

  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:PossibleObstacleSpots", "drawingOnImage");

    for(int i = 0; i < scanlineCount; ++i)
    {
      const Scanline& line = scanlines[i];
      for(int j = 0; j < line.spotCount; ++j)
      {
        CROSS("representation:PossibleObstacleSpots", line.xImg, line.spots[j], 2, 2, Drawings::ps_solid, ColorClasses::red);
      }
    }
  }
};
