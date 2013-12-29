/**
* @file EvidenceGrid.cpp
* The file implements a class that represents an evidence cells.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "EvidenceGrid.h"
#include "Platform/BHAssert.h"
#include <cstring>

EvidenceGrid::EvidenceGrid(float width, int widthInCells, int heightInCells, const Cell& empty)
: Grid(width, widthInCells, heightInCells),
  empty(empty),
  cells(widthInCells * heightInCells, empty)
{
}

void EvidenceGrid::clear()
{
  cells.assign(cells.size(), empty);
}

void EvidenceGrid::move(const Vector2<>& newCenter)
{
  Vector2<> oldCenter = center;
  center = Vector2<>(std::floor(newCenter.x / cellSize + 0.5f) * cellSize,
                     std::floor(newCenter.y / cellSize + 0.5f) * cellSize); // here we add +0.5 to go to the closest
  Vector2<int> offset(int((center.x - oldCenter.x) / cellSize),
                      int((center.y - oldCenter.y) / cellSize));

  if(abs(offset.x) >= widthInCells || abs(offset.y) >= heightInCells)
    clear();
  else if(offset.x < 0)
    if(offset.y < 0)
    {
      copy(0, 0, -offset.x, -offset.y, widthInCells + offset.x, heightInCells + offset.y);
      clear(0, 0, widthInCells, -offset.y);
      clear(0, -offset.y, -offset.x, heightInCells + offset.y);
    }
    else if(offset.y > 0)
    {
      copy(0, offset.y, -offset.x, 0, widthInCells + offset.x, heightInCells - offset.y);
      clear(0, heightInCells - offset.y, widthInCells, offset.y);
      clear(0, 0, -offset.x, heightInCells - offset.y);
    }
    else
    {
      copy(0, 0, -offset.x, 0, widthInCells + offset.x, heightInCells);
      clear(0, 0, -offset.x, heightInCells);
    }
  else if(offset.x > 0)
    if(offset.y < 0)
    {
      copy(offset.x, 0, 0, -offset.y, widthInCells - offset.x, heightInCells + offset.y);
      clear(0, 0, widthInCells, -offset.y);
      clear(widthInCells - offset.x, -offset.y, offset.x, heightInCells + offset.y);
    }
    else if(offset.y > 0)
    {
      copy(offset.x, offset.y, 0, 0, widthInCells - offset.x, heightInCells - offset.y);
      clear(0, heightInCells - offset.y, widthInCells, offset.y);
      clear(widthInCells - offset.x, 0, offset.x, heightInCells - offset.y);
    }
    else
    {
      copy(offset.x, 0, 0, 0, widthInCells - offset.x, heightInCells);
      clear(widthInCells - offset.x, 0, offset.x, heightInCells);
    }
  else if(offset.y < 0)
  {
    copy(0, 0, 0, -offset.y, widthInCells, heightInCells + offset.y);
    clear(0, 0, widthInCells, -offset.y);
  }
  else if(offset.y > 0)
  {
    copy(0, offset.y, 0, 0, widthInCells, heightInCells - offset.y);
    clear(0, heightInCells - offset.y, widthInCells, offset.y);
  }
}

void EvidenceGrid::copy(int fromX, int fromY, int toX, int toY, int sizeX, int sizeY)
{
  ASSERT(fromX >= 0 && fromX + sizeX <= widthInCells);
  ASSERT(fromY >= 0 && fromY + sizeY <= heightInCells);
  ASSERT(toX >= 0 && toX + sizeX <= widthInCells);
  ASSERT(toY >= 0 && toY + sizeY <= heightInCells);
  if(toY < fromY)
    for(int y = 0; y < sizeY; ++y)
      memmove(&(*this)[toY + y][toX], &(*this)[fromY + y][fromX], sizeX * sizeof(Cell));
  else
    for(int y = sizeY - 1; y >= 0; --y)
      memmove(&(*this)[toY + y][toX], &(*this)[fromY + y][fromX], sizeX * sizeof(Cell));
}

void EvidenceGrid::clear(int startX, int startY, int sizeX, int sizeY)
{
  ASSERT(startX >= 0 && startX + sizeX <= widthInCells);
  ASSERT(startY >= 0 && startY + sizeY <= heightInCells);
  for(int y = 0; y < sizeY; ++y)
    for(Cell* c = (*this)[startY + y], *cEnd = c + sizeX; c < cEnd; ++c)
      *c = empty;
}
