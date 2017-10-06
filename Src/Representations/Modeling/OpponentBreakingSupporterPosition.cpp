/**
 * @file OpponentBreakingSupporterPosition.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "OpponentBreakingSupporterPosition.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"

void OpponentBreakingSupporterPosition::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:OpponentBreakingSupporterPosition:position", "drawingOnField");

  COMPLEX_DRAWING("representation:OpponentBreakingSupporterPosition:position")
  {
    CROSS("representation:OpponentBreakingSupporterPosition:position", asObstacle.center.x(), asObstacle.center.y(), 20, 70, Drawings::solidPen, ColorRGBA::magenta);
    CIRCLE("representation:OpponentBreakingSupporterPosition:position", asObstacle.center.x(), asObstacle.center.y(), 500, 40, Drawings::solidPen, ColorRGBA::magenta, Drawings::noBrush, ColorRGBA());
  }
}
