/**
* @file Grid.cpp
* The file implements the base class for grids.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Grid.h"
#include "Platform/BHAssert.h"

Grid::Grid()
: width(0),
  height(0),
  cellSize(0),
  widthInCells(0),
  heightInCells(0)
{
}

Grid::Grid(float width, int widthInCells, int heightInCells)
: width(width),
  widthInCells(widthInCells),
  heightInCells(heightInCells)
{
  ASSERT(width != 0.f && widthInCells && heightInCells);
  cellSize = width / widthInCells;
  height = heightInCells * cellSize;
}
