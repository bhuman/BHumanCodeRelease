/**
* @file GlobalFieldCoverage.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "GlobalFieldCoverage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"

GlobalFieldCoverage::Cell::Cell(const unsigned timestamp, const unsigned value,
                                const float positionOnFieldX, const float positionOnFieldY,
                                const float cellLengthX, const float cellLengthY)
  : timestamp(timestamp), value(value), positionOnField(positionOnFieldX, positionOnFieldY)
{
  const float xMin = positionOnFieldX - cellLengthX / 2.f;
  const float xMax = positionOnFieldX + cellLengthX / 2.f;
  const float yMin = positionOnFieldY - cellLengthY / 2.f;
  const float yMax = positionOnFieldY + cellLengthY / 2.f;

  polygon[0] = Vector2f(xMin + 25.0f, yMin + 25.0f);
  polygon[1] = Vector2f(xMin + 25.0f, yMax - 25.0f);
  polygon[2] = Vector2f(xMax - 25.0f, yMax - 25.0f);
  polygon[3] = Vector2f(xMax - 25.0f, yMin + 25.0f);
}

void GlobalFieldCoverage::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GlobalFieldCoverage:coverage", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalFieldCoverage:coverageNumbers", "drawingOnField");
  COMPLEX_DRAWING("representation:GlobalFieldCoverage:coverage")
  {
    if(Blackboard::getInstance().exists("FrameInfo"))
    {
      const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);

      auto alpha = [&](const int lastSeen)
      {
        const int alpha = (theFrameInfo.time - lastSeen) / 300;
        return static_cast<unsigned char>(std::min(alpha, 255));
      };

      for(size_t i = 0; i < grid.size(); ++i)
      {
        const Cell& c = grid[i];
        const unsigned char a = alpha(c.value);
        const ColorRGBA color(255, 0, 0, a);
        QUADRANGLE("representation:GlobalFieldCoverage:coverage",
                   c.polygon[0].x(), c.polygon[0].y(),
                   c.polygon[1].x(), c.polygon[1].y(),
                   c.polygon[2].x(), c.polygon[2].y(),
                   c.polygon[3].x(), c.polygon[3].y(),
                   20, Drawings::solidPen, color);

        DRAWTEXT("representation:GlobalFieldCoverage:coverageNumbers", c.polygon[0].x(), c.polygon[0].y(), 100, ColorRGBA(255, 255, 255, 255), a);
      }
    }
  }

  DECLARE_DEBUG_DRAWING("representation:GlobalFieldCoverage:worstCell", "drawingOnField");
  COMPLEX_DRAWING("representation:GlobalFieldCoverage:worstCell")
  {
    unsigned worst;
    const Cell* worstCell = nullptr;
    for(const auto& c : grid)
    {
      if(!worstCell || c.value < worst)
      {
        worst = c.value;
        worstCell = &c;
      }
    }

    if(worstCell)
    {
      CROSS("representation:GlobalFieldCoverage:worstCell",
            worstCell->positionOnField.x(), worstCell->positionOnField.y(), 75, 30,
            Drawings::solidPen, ColorRGBA(255, 192, 203));
    }
  }
}