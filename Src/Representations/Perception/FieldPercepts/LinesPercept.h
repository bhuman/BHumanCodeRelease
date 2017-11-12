#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Geometry.h"
#include <vector>

/**
 * @struct LinesPercept
 * This struct contains all the linespots
 */
STREAMABLE(LinesPercept,
{
  STREAMABLE(Line,
  {
    std::vector<int> spotInImgHeights; /**<The height of each spot in the image (only needed for validation) */
    Line() = default;
    Line(const Vector2f& base, const Vector2f& direction) : line(base, direction) {},

    (Geometry::Line) line, /**<The fitted line in field coordinates */
    (std::vector<Vector2f>) spotsInField, /**< Spots that are on this line  (in relative field coordinates). NOT FITTED TO LINE*/
    (std::vector<Vector2i>) spotsInImg, /**< Spots that are on this line  (in image coordinates). NOT FITTED TO LINE*/
    (Vector2i) firstImg,/**<First spot of the line in image coordinates, NOT FITTED TO LINE*/
    (Vector2i) lastImg,/**<Last spot of the line in image coordinates, NOT FITTED TO LINE*/
    (Vector2f) firstField, /**<start of the fitted line in field coordinates */
    (Vector2f) lastField,/**< end of the fitted line in field coordinates */
    (bool)(false) belongsToCircle,
  });

  /**
   * The method draws all line spots.
   */
  void draw() const,
  (std::vector<Line>) lines,
});

inline void LinesPercept::draw() const
{
  static const ColorRGBA colors[8] =
  {
    ColorRGBA::black, ColorRGBA::blue, ColorRGBA::cyan, ColorRGBA::violet,
    ColorRGBA::green, ColorRGBA::magenta, ColorRGBA::orange, ColorRGBA::gray
  };

  DECLARE_DEBUG_DRAWING("representation:LinesPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:LinesPercept:field", "drawingOnField");

  COMPLEX_DRAWING("representation:LinesPercept:field")
  {
    int colorIndex = 0;
    for(const Line& line : lines)
    {
      const Vector2f p1 = line.line.base;
      const Vector2f p2 = line.line.base + (line.line.direction);
      const ColorRGBA color = colors[colorIndex];
      LINE("representation:LinesPercept:field", p1.x(), p1.y(), p2.x(), p2.y(), 20, Drawings::solidPen, color);

      for(const auto& spot : line.spotsInField)
      {
        CROSS("representation:LinesPercept:field", spot.x(), spot.y(), 15, 10, Drawings::solidPen, color);
      }

      colorIndex = (colorIndex + 1) % 8;
    }
  }

  COMPLEX_DRAWING("representation:LinesPercept:image")
  {
    int colorIndex = 0;
    for(const Line& line : lines)
    {
      const ColorRGBA color = colors[colorIndex];
      LINE("representation:LinesPercept:image", line.firstImg.x(), line.firstImg.y(),
           line.lastImg.x(), line.lastImg.y(), 2, Drawings::solidPen, color);
      for(const auto& spot : line.spotsInImg)
      {
        CROSS("representation:LinesPercept:image", spot.x(), spot.y(), 2, 1, Drawings::solidPen, color);
      }
      colorIndex = (colorIndex + 1) % 8;
    }
  }
}
