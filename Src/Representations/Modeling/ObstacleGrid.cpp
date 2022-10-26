/**
 * @file ObstacleGrid.cpp
 *
 * @author Nele Matschull
 */

#include "ObstacleGrid.h"
#include "Debugging/DebugDrawings.h"

//TODO use parameters instead of numbers

void ObstacleGrid::draw()
{
  DECLARE_DEBUG_DRAWING("representation:ObstacleGrid", "drawingOnField");
  ColorRGBA free = ColorRGBA(5, 196, 230, 200);
  ColorRGBA occupied = ColorRGBA(230, 5, 5, 200);
  Vector2f topLeft = Vector2f(600, 600);
  for(int i = 0; i < 30; ++i)
  {
    for(int j = 0; j < 30; ++j)
    {
      RECTANGLE2("representation:ObstacleGrid", topLeft, 40, 40, 0, 5, Drawings::noPen, ColorRGBA(),
                 Drawings::solidBrush, cells[30 * i + j].state > 0 ? occupied : free);
      topLeft.y() -= 40;
    }
    topLeft.x() -= 40;
    topLeft.y() = 600;
  }
}

int ObstacleGrid::worldToGrid(const Vector2f& inWorld) const
{
  Vector2f inGrid = -gridOffset * inWorld / 60 + Vector2f(45 / 2, 45 / 2);
  return 2025 - 1 - (int)inGrid.x() * 45 - (int)inGrid.y();
}
