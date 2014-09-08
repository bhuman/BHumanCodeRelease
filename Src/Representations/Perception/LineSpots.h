#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Vector.h"
#include "Tools/Math/Geometry.h"
#include <vector>

/**
* @class LineSpots
* This class contains all the linespots
*/
STREAMABLE(LineSpots,
{
public:
  STREAMABLE(Line,
  {
  public:
    Line() = default;
    Line(const Vector2<float>& base, const Vector2<float>& direction) : line(base, direction) {},

    (Geometry::Line) line, /**<The fitted line in field coordinates */
    (std::vector<Vector2<>>) spotsInField, /**< Spots that are on this line  (in relative field coordinates). NOT FITTED TO LINE*/
    (std::vector<Vector2<int>>) spotsInImg, /**< Spots that are on this line  (in image coordinates). NOT FITTED TO LINE*/
    (std::vector<int>) spotInImgHeights, /**<The height of each spot in the image (only needed for validation) */
    (Vector2<int>) firstImg,/**<First spot of the line in image coordinates, NOT FITTED TO LINE*/
    (Vector2<int>) lastImg,/**<Last spot of the line in image coordinates, NOT FITTED TO LINE*/
    (Vector2<>) firstField, /**<start of the fitted line in field coordinates */
    (Vector2<>) lastField,/**< end of the fitted line in field coordinates */
    (bool)(false) belongsToCircle,
  });
  
  STREAMABLE(Intersection,
  {
  public:
    ENUM(IntersectionType,
      L,
      T,
      X
    ),
    (IntersectionType) type,
    (Vector2<>) pos, /**< The fieldcoordinates of the intersection */
  });
  
  
  /**
  * The method draws all line spots.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:LineSpots:image", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:LineSpots:field", "drawingOnField");
    
    COMPLEX_DRAWING("representation:LineSpots:field",
    {
     
      ColorRGBA colors[8] = {ColorRGBA::black, ColorRGBA::blue, ColorRGBA::cyan, ColorRGBA::gray,
                             ColorRGBA::green, ColorRGBA::magenta, ColorRGBA::orange, ColorRGBA::violet};
      int colorIndex = 0;
      for(const Line& line : lines)
      {
        Vector2<> p1 = line.line.base + (line.line.direction * 2);
        Vector2<> p2 = line.line.base - (line.line.direction * 2);
        ColorRGBA color = colors[colorIndex];
        LINE("representation:LineSpots:field", p1.x, p1.y, p2.x, p2.y, 20, Drawings::ps_dot, color);
        LINE("representation:LineSpots:field", line.spotsInField[0].x,
             line.spotsInField[0].y, line.spotsInField.back().x, line.spotsInField.back().y, 30, Drawings::ps_solid, color);

        for(const auto& spot : line.spotsInField)
        {
          CROSS("representation:LineSpots:field", spot.x, spot.y, 15, 10, Drawings::ps_solid, color);
        }
        
        CROSS("representation:LineSpots:field", line.firstField.x, line.firstField.y, 30, 15, Drawings::ps_solid, color);
        CROSS("representation:LineSpots:field", line.lastField.x, line.lastField.y, 30, 15, Drawings::ps_solid, color);
        colorIndex = (colorIndex + 1) % 8;
      }
      
      if(circleSeen)
      {
        CROSS("representation:LineSpots:field", circle.center.x, circle.center.y, 60, 30, Drawings::ps_solid, ColorRGBA::green);
        CIRCLE("representation:LineSpots:field", circle.center.x, circle.center.y, circle.radius, 30, Drawings::ps_solid, ColorRGBA::blue, Drawings::bs_null, ColorRGBA::blue);
      }
      
      for(const Intersection& intersection : intersections)
      {
        CROSS("representation:LineSpots:field", intersection.pos.x, intersection.pos.y, 60, 30, Drawings::ps_solid, ColorRGBA::blue);
      }      
    });
    
    COMPLEX_DRAWING("representation:LineSpots:image",
    {
      
      ColorRGBA colors[8] = {ColorRGBA::black, ColorRGBA::blue, ColorRGBA::cyan, ColorRGBA::gray,
                             ColorRGBA::green, ColorRGBA::magenta, ColorRGBA::orange, ColorRGBA::violet};
      int colorIndex = 0;
      for(const Line& line : lines)
      {
        ColorRGBA color = colors[colorIndex];
        LINE("representation:LineSpots:image", line.spotsInImg[0].x, line.spotsInImg[0].y,
             line.spotsInImg.back().x, line.spotsInImg.back().y, 2, Drawings::ps_solid, color);
        for(const auto& spot : line.spotsInImg)
        {
          CROSS("representation:LineSpots:image", spot.x, spot.y, 2, 1, Drawings::ps_solid, color);
        }
        CROSS("representation:LineSpots:image", line.firstImg.x, line.firstImg.y, 5, 3, Drawings::ps_solid, color);
        CROSS("representation:LineSpots:image", line.lastImg.x, line.lastImg.y, 5, 3, Drawings::ps_solid, color);
        colorIndex = (colorIndex + 1) % 8;
      }
    });
  },

  /**line spots in image coordinates*/
  (std::vector<Line>) lines,
  (std::vector<Intersection>) intersections,
  (unsigned) spotCount, /**<Total number of detected spots */
  (Geometry::Circle) circle, /**< location and radius of the circle if found */
  (bool) circleSeen, /**< True if circle has been found this frame */

});
