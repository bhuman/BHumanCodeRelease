#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Geometry.h"
#include <vector>

/**
 * @struct LineSpots
 * This struct contains all the linespots
 */
STREAMABLE(LineSpots,
{
  STREAMABLE(Line,
  {
    Line() = default;
    Line(const Vector2f& base, const Vector2f& direction) : line(base, direction) {},

    (Geometry::Line) line, /**<The fitted line in field coordinates */
    (std::vector<Vector2f>) spotsInField, /**< Spots that are on this line  (in relative field coordinates). NOT FITTED TO LINE*/
    (std::vector<Vector2i>) spotsInImg, /**< Spots that are on this line  (in image coordinates). NOT FITTED TO LINE*/
    (std::vector<int>) spotInImgHeights, /**<The height of each spot in the image (only needed for validation) */
    (Vector2i) firstImg,/**<First spot of the line in image coordinates, NOT FITTED TO LINE*/
    (Vector2i) lastImg,/**<Last spot of the line in image coordinates, NOT FITTED TO LINE*/
    (Vector2f) firstField, /**<start of the fitted line in field coordinates */
    (Vector2f) lastField,/**< end of the fitted line in field coordinates */
    (bool)(false) belongsToCircle,
  });

  STREAMABLE(Intersection,
  {
    ENUM(IntersectionType,
    {,
      L,
      T,
      X,
    }),
    
    (IntersectionType) type,
    (Vector2f) pos, /**< The fieldcoordinates of the intersection */
    (Vector2f) dir1, /**< The first direction of the lines intersected. (In field coordinates) */
    (Vector2f) dir2, /**< The second direction of the lines intersected. (In field coordinates) */
    (Line) line1, /**< The first line of the intersection*/
    (Line) line2, /**< The second line of the intersection*/
  });

  /**
   * The method draws all line spots.
   */
  void draw() const,

  /**line spots in image coordinates*/
  (std::vector<Line>) lines,
  (std::vector<Intersection>) intersections,
  (unsigned) spotCount, /**<Total number of detected spots */
  (Geometry::Circle) circle, /**< location and radius of the circle if found */
  (bool) circleSeen, /**< True if circle has been found this frame */
});

inline void LineSpots::draw() const
{
  static const ColorRGBA colors[8] =
  {
    ColorRGBA::black, ColorRGBA::blue, ColorRGBA::cyan, ColorRGBA::gray,
    ColorRGBA::green, ColorRGBA::magenta, ColorRGBA::orange, ColorRGBA::violet
  };

  DECLARE_DEBUG_DRAWING("representation:LineSpots:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:LineSpots:field", "drawingOnField");

  COMPLEX_DRAWING("representation:LineSpots:field")
  {
    int colorIndex = 0;
    for(const Line& line : lines)
    {
      const Vector2f p1 = line.line.base + (line.line.direction * 2);
      const Vector2f p2 = line.line.base - (line.line.direction * 2);
      const ColorRGBA color = colors[colorIndex];
      LINE("representation:LineSpots:field", p1.x(), p1.y(), p2.x(), p2.y(), 20, Drawings::dottedPen, color);
      LINE("representation:LineSpots:field", line.spotsInField[0].x(),
           line.spotsInField[0].y(), line.spotsInField.back().x(), line.spotsInField.back().y(), 30, Drawings::solidPen, color);

      for(const auto& spot : line.spotsInField)
      {
        CROSS("representation:LineSpots:field", spot.x(), spot.y(), 15, 10, Drawings::solidPen, color);
      }

      CROSS("representation:LineSpots:field", line.firstField.x(), line.firstField.y(), 30, 15, Drawings::solidPen, color);
      CROSS("representation:LineSpots:field", line.lastField.x(), line.lastField.y(), 30, 15, Drawings::solidPen, color);
      colorIndex = (colorIndex + 1) % 8;
    }

    if(circleSeen)
    {
      CROSS("representation:LineSpots:field", circle.center.x(), circle.center.y(), 60, 30, Drawings::solidPen, ColorRGBA::green);
      CIRCLE("representation:LineSpots:field", circle.center.x(), circle.center.y(), circle.radius, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
    }

    for(const Intersection& intersection : intersections)
    {
      CROSS("representation:LineSpots:field", intersection.pos.x(), intersection.pos.y(), 60, 30, Drawings::solidPen, ColorRGBA::blue);
    }
  }

  COMPLEX_DRAWING("representation:LineSpots:image")
  {
    int colorIndex = 0;
    for(const Line& line : lines)
    {
      const ColorRGBA color = colors[colorIndex];
      LINE("representation:LineSpots:image", line.spotsInImg[0].x(), line.spotsInImg[0].y(),
           line.spotsInImg.back().x(), line.spotsInImg.back().y(), 2, Drawings::solidPen, color);
      for(const auto& spot : line.spotsInImg)
      {
        CROSS("representation:LineSpots:image", spot.x(), spot.y(), 2, 1, Drawings::solidPen, color);
      }
      CROSS("representation:LineSpots:image", line.firstImg.x(), line.firstImg.y(), 5, 3, Drawings::solidPen, color);
      CROSS("representation:LineSpots:image", line.lastImg.x(), line.lastImg.y(), 5, 3, Drawings::solidPen, color);
      colorIndex = (colorIndex + 1) % 8;
    }
  }
}
