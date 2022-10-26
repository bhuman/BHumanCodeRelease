/**
 * @file ObstacleGridProvider.cpp
 *
 * @author Nele Matschull
 *
 * based on USObstacleGridProvider from 2013
 */

#include "ObstacleGridProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Module.h"

MAKE_MODULE(ObstacleGridProvider, modeling);

void ObstacleGridProvider::update(ObstacleGrid& theObstacleGrid)
{
  if(!initialized)
  {
    lastOdometry = theOdometryData;
    cells = theObstacleGrid.cells;
    initialized = true;
  }

  if(!theGameState.isPenalized() && theFrameInfo.getTimeSince(lastTimePenalized) > 3000)
  {
    moveGrid();
    ageCellState();
  }
  else if(theGameState.isPenalized())
  {
    lastTimePenalized = theFrameInfo.time;
  }

  theObstacleGrid.gridOffset = -accumulatedOdometry;

  Pose2f origin = theRobotPose + theObstacleGrid.gridOffset;

  DECLARE_DEBUG_DRAWING("module:ObstacleGrid:field", "drawingOnField");

  std::vector<unsigned char> debugCells;
  debugCells.reserve(cellCount * cellCount);

  float distanceToGridBorder = cellCount / 2.f * cellSize - cellCount / 2.f;

  //this is a temporary solution, currently the obstacles of the local model are used
  for (const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    //TODO Switch obstacle.getRobotDepth() with the actual size of the obstacle

    Vector2f center = obstacle.center;
    Vector2f startPos = Vector2f(std::clamp(center.x() - obstacle.getRobotDepth(), -distanceToGridBorder, distanceToGridBorder),
      std::clamp(center.y() - obstacle.getRobotDepth(), -distanceToGridBorder, distanceToGridBorder));
    Vector2f endPos = Vector2f(std::clamp(center.x() + obstacle.getRobotDepth(), -distanceToGridBorder, distanceToGridBorder),
      std::clamp(center.y() + obstacle.getRobotDepth(), -distanceToGridBorder, distanceToGridBorder));

    for (float x = startPos.x(); x <= endPos.x(); x++)
    {
      for (float y = startPos.y(); y <= endPos.y(); y++)
      {
        float c = x - center.x();
        float d = y - center.y();
        if ((c * c + d * d) <= squaredGoalPostRadius)
        {
          int idx = worldToGrid(Vector2f(x, y));
          Cell* c = &cells[idx];
          c->state++;
          c->lastUpdate = theFrameInfo.time;
        }
      }
    }
  }

  for (int i = 0; i < cellCount*cellCount; ++i)
  {
    debugCells.push_back(cells[i].state);
  }
  GRID_MONO("module:ObstacleGrid:field", theRobotPose.translation.x(), theRobotPose.translation.y(), cellSize, cellCount, cellCount, ColorRGBA(255, 0, 0, 200), debugCells.data());
}

void ObstacleGridProvider::moveGrid()
{
  accumulatedOdometry += theOdometryData - lastOdometry;
  lastOdometry = theOdometryData;
  // Move grid backwards in x direction (robot moves forward):
  if(accumulatedOdometry.translation.x() >= cellSize)
  {
    accumulatedOdometry.translation.x() -= cellSize;
    for(int y = 0; y < cellCount; ++y)
    {
      Cell* cStartNew = &cells[y * cellCount];
      Cell* cStartOld = cStartNew + 1;
      memmove(cStartNew, cStartOld, sizeof(Cell) * (cellCount - 1));
      cStartNew[cellCount - 1] = Cell();
    }
  }
  // Move grid forward in x direction (robot moves backwards):
  else if(accumulatedOdometry.translation.x() <= -cellSize)
  {
    accumulatedOdometry.translation.x() += cellSize;
    for(int y = 0; y < cellCount; ++y)
    {
      Cell* cStartOld = &cells[y * cellCount];
      Cell* cStartNew = cStartOld + 1;
      memmove(cStartNew, cStartOld, sizeof(Cell) * (cellCount - 1));
      cStartOld[0] = Cell();
    }
  }
  // Move grid backwards in y direction (robot moves to the left):
  if(accumulatedOdometry.translation.y() >= cellSize)
  {
    accumulatedOdometry.translation.y() -= cellSize;
    Cell* cStartOld = &cells[cellCount];
    Cell* cStartNew = &cells[0];
    memmove(cStartNew, cStartOld, sizeof(Cell) * cellCount * (cellCount - 1));
    Cell* c = &cells[(cellCount - 1) * cellCount];
    for(int x = 0; x < cellCount; ++x)
      c[x] = Cell();
  }
  // Move grid forward in y direction (robot moves to the right):
  else if(accumulatedOdometry.translation.y() <= -cellSize)
  {
    accumulatedOdometry.translation.y() += cellSize;
    Cell* cStartNew = &cells[cellCount];
    Cell* cStartOld = &cells[0];
    memmove(cStartNew, cStartOld, sizeof(Cell) * cellCount * (cellCount - 1));
    Cell* c = &cells[0];
    for(int x = 0; x < cellCount; ++x)
      c[x] = Cell();
  }
}

void ObstacleGridProvider::ageCellState()
{
  for (int i = 0; i < cellCount * cellCount; ++i)
  {
    Cell& c = cells[i];
    if (c.state)
    {
      if (theFrameInfo.getTimeSince(c.lastUpdate) > cellFreeInterval)
      {
        c.state--;
      }
    }
  }
}

int ObstacleGridProvider::worldToGrid(const Vector2f& inWorld) const
{
  Vector2f inGrid = ((theRobotPose * inWorld) - theRobotPose.translation) / cellSize + Vector2f(cellCount / 2, cellCount / 2);
  return (int)inGrid.y() * cellCount + (int)inGrid.x();
}
