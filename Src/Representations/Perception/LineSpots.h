/**
* @file LineSpots.h
* Declaration of a class that represents a spot that  indicates a line.
* @author jeff
*/

#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Streams/AutoStreamable.h"
#include <algorithm>

/**
* @class LineSpots
* This class contains all the linespots and nonlinesposts (=white regions which are no lines)
*/
STREAMABLE(LineSpots,
{
public:
  /**
   * @class LineSpot
   * A class that represents a spot that's an indication of a line.
   */
  STREAMABLE(LineSpot,
  {
  public:
    bool operator<(const LineSpots::LineSpot& ls) const
    {
      if(std::min(p1.x, p2.x) < std::min(ls.p1.x, ls.p2.x))
        return true;
      else if(std::min(p1.y, p2.y) < std::min(ls.p1.y, ls.p2.y))
        return true;
      else
        return false;
    }

    bool operator<(const LineSpots::LineSpot* ls) const
    {
      return *this < *ls;
    },

    (float) alpha, /**< the direction/rotation of the region   | */
    (float) alphaLen, /**< "ausbreitung entlang alpha"         |-> Haupttraegheitsachsenbla */
    (float) alphaLen2, /**< "ausbreitung orthogonal zu alpha"  | */
    (int) xs, /**< center of mass x */
    (int) ys, /**< center of mass y */
    (Vector2<int>) p1,
    (Vector2<int>) p2, /**< The starting/end point of this linespot in image coordinates*/
  });

  /**
   * @class NonLineSpot
   * This class represents a white region which is no line
   */
  STREAMABLE(NonLineSpot,
  {,
    (Vector2<int>) p1, /**< start point of this spot in image coordinates */
    (Vector2<int>) p2, /**< end point of this spot in image coordinates */
    (int) size, /**< The size of the coresponding region in the image */
  });

  /**
  * The method draws all line spots.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:LineSpots:nonLineSpots", "drawingOnImage");
    COMPLEX_DRAWING("representation:LineSpots:nonLineSpots",
    {
      for(std::vector<LineSpots::NonLineSpot>::const_iterator i = nonLineSpots.begin(); i != nonLineSpots.end(); ++i)
      {
        ARROW("representation:LineSpots:nonLineSpots", i->p1.x, i->p1.y, i->p2.x, i->p2.y, 2, Drawings::ps_solid, ColorClasses::blue);
      }
    });

    DECLARE_DEBUG_DRAWING("representation:LineSpots:Image", "drawingOnImage"); // Draws the LineSpots to the image
    COMPLEX_DRAWING("representation:LineSpots:Image",
    {
      for(std::vector<LineSpots::LineSpot>::const_iterator i = spots.begin(); i != spots.end(); ++i)
      {
        LINE("representation:LineSpots:Image", i->xs, i->ys, i->xs + (int)(cos(i->alpha + pi_2) * i->alphaLen2), i->ys + (int)(sin(i->alpha + pi_2)*i->alphaLen2), 0, Drawings::ps_solid, ColorClasses::black);
        ARROW("representation:LineSpots:Image", i->p1.x, i->p1.y, i->p2.x, i->p2.y, 0, Drawings::ps_solid, ColorClasses::black);
      }
    });
  },

  (std::vector<LineSpot>) spots, /**< All the line spots */
  (std::vector<NonLineSpot>) nonLineSpots, /**< All the non line spots (= white regions which are no lines)*/
});
