/**
 * @file BallSearchAreasProvider.cpp
 *
 * This file implements a grid for the ball search.
 *
 * @author Sina Schreiber
 */

#include "BallSearchAreasProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Math/BHMath.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include <algorithm>

MAKE_MODULE(BallSearchAreasProvider);

BallSearchAreasProvider::BallSearchAreasProvider()
{
  initializeGrid();
}

void BallSearchAreasProvider::initializeGrid()
{
  grid.clear();
  cellCountX = static_cast<unsigned>(std::abs(theFieldDimensions.xPosOwnGoalLine) * 2 / cellWidth + 1);
  cellCountY = static_cast<unsigned>(std::abs(theFieldDimensions.yPosRightTouchline) * 2 / cellHeight + 1);
  // building the grid from the upper left to the bottom right corner of the field
  for(unsigned y = 0; y < cellCountY; y++)
  {
    for(unsigned x = 0; x < cellCountX; x++)
    {
      const Vector2f positionOfCell(theFieldDimensions.xPosOwnGoalLine + x * cellWidth, theFieldDimensions.yPosRightTouchline + y * cellHeight);
      grid.emplace_back(positionOfCell, 0);
    }
  }
  ASSERT(!grid.empty());
  colorVector.reserve(cellCountX * cellCountY);
}

void BallSearchAreasProvider::update(BallSearchAreas& theBallSearchAreas)
{
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:heatmap", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:fullGrid", "drawingOnField");

  DEBUG_RESPONSE_ONCE("module:BallSearchAreasProvider:reinitializeGrid")
    initializeGrid();

  ASSERT(grid.size() == cellCountX * cellCountY);
  reset();
  STOPWATCH("module:BallSearchAreasProvider:updateCells")
    updateCells();
  STOPWATCH("module:BallSearchAreasProvider:draw")
    draw();
  theBallSearchAreas.cellToSearchNext = [this](const Agent& agent) { return positionCellToSearchNext(getVoronoiGrid(agent)); };
  theBallSearchAreas.filterCellsToSearch = [this](const Agent& agent) { return getVoronoiGrid(agent); };
  theBallSearchAreas.grid = grid;
}

void BallSearchAreasProvider::draw()
{
  // ID - center, time - bottom left, priority - top left
  colorVector.clear();

  COMPLEX_DRAWING("module:BallSearchAreasProvider:heatmap")
  {
    for(unsigned i = 0; i < cellCountX * cellCountY; i++)
    {
      RECTANGLE("module:BallSearchAreasProvider:heatmap", grid[i].positionOnField.x() + cellWidth / 2, grid[i].positionOnField.y() - cellHeight / 2, grid[i].positionOnField.x() - cellWidth / 2, grid[i].positionOnField.y() + cellHeight / 2, 20, Drawings::solidPen, ColorRGBA::black);

      const unsigned timeLastSeen = theFrameInfo.time - grid[i].timestamp;
      const unsigned char alpha = static_cast<unsigned char>((heatmapMaxTime - timeLastSeen) / heatmapMaxTime * heatmapAlpha);
      // Interpolation between black(> heatmapMaxTime) - yellow(0).
      const ColorRGBA cellColor = timeLastSeen > heatmapMaxTime ? ColorRGBA(0, 0, 0, heatmapAlpha) : ColorRGBA(255, 255, 0, alpha).blend(ColorRGBA(0, 0, 0, heatmapAlpha - alpha));
      colorVector.push_back(cellColor);

      //Draw time
      DRAW_TEXT("module:BallSearchAreasProvider:heatmap", grid[i].positionOnField.x() - cellWidth / 2, grid[i].positionOnField.y() - cellHeight / 2, 100, ColorRGBA::white, timeLastSeen * grid[i].priority);
    }
  }

  COMPLEX_DRAWING("module:BallSearchAreasProvider:fullGrid")
  {
    const Vector2f startPosition = theRobotPose.translation;
    DRAW_SECTOR_WHEEL("module:BallSearchAreasProvider:fullGrid", calculateObstacleSectors(), startPosition);
    for(unsigned i = 0; i < cellCountX * cellCountY; i++)
    {
      const int timeLastSeen = theFrameInfo.time - grid[i].timestamp;
      //Draw time
      DRAW_TEXT("module:BallSearchAreasProvider:fullGrid", grid[i].positionOnField.x() - cellWidth / 2, grid[i].positionOnField.y() - cellHeight / 2, 100, ColorRGBA::white, timeLastSeen * grid[i].priority);
      //Draw Priority, if priority is larger than 1.
      if(grid[i].priority > 1)
      {
        DRAW_TEXT("module:BallSearchAreasProvider:fullGrid", grid[i].positionOnField.x(), grid[i].positionOnField.y(), 100, ColorRGBA::black, grid[i].priority);
      }
    }
  }
  GRID_RECTANGLE_RGBA("module:BallSearchAreasProvider:heatmap", 0, 0, cellWidth, cellHeight, cellCountX, cellCountY, colorVector.data());
}

void BallSearchAreasProvider::updateTimestamp(BallSearchAreas::Cell& cell)
{
  cell.timestamp = theFrameInfo.time;
}

bool BallSearchAreasProvider::cellInView(const BallSearchAreas::Cell& cell, const std::vector<Vector2f>& p) const
{
  // return if the cell is in the view of the robot by checking if the distance to the cell is small enough and if the cell is insight of the polygon of visible field parts.
  // the cast to int is justified because the size will always be 4
  return returnDistanceToCellSqr(cell) < sqr(maxDistanceToCell) && Geometry::isPointInsideConvexPolygon(p.data(), static_cast<int>(p.size()), cell.positionOnField);
}

const std::vector<BallSearchAreas::Cell> BallSearchAreasProvider::getVoronoiGrid(const Agent& agent) const
{
  std::vector<BallSearchAreas::Cell> voronoiGrid;
  voronoiGrid.reserve(grid.size());

  // Check if the current cell is part of the voronoi grid, return the cells inside of the voronoi grid.
  for(const auto& cell : grid)
  {
    if(Geometry::isPointInsidePolygon(cell.positionOnField, agent.baseArea))
    {
      voronoiGrid.emplace_back(cell);
    }
  }
  return voronoiGrid;
}

float BallSearchAreasProvider::returnDistanceToCellSqr(const BallSearchAreas::Cell& cell) const
{
  return (cell.positionOnField - theRobotPose.translation).squaredNorm();
}

void BallSearchAreasProvider::reset()
{
  if(theGameState.timeWhenStateStarted == theFrameInfo.time && (theGameState.isCornerKick() || theGameState.isKickIn() || theGameState.isGoalKick() || theGameState.isKickOff()))
  {
    for(auto& cell : grid)
    {
      cell.timestamp = theFrameInfo.time;
    }
  }
}

const Vector2f BallSearchAreasProvider::positionCellToSearchNext(const std::vector<BallSearchAreas::Cell>& gridToSearch) const
{
  ASSERT(gridToSearch.size() != 0);
  BallSearchAreas::Cell nextSearchCell = gridToSearch[0];

  for(const auto& cell : gridToSearch)
  {
    // Saves the cell with the highest search score, which is calculatet by: If the difference between the timestamp of the cell and the timestamp of the cameraframe multiplied by the priority (evaluation of how long a cell has not been seen multiplied by the importance of the cell in the current gamestate) is higher than the stored value, replace the stored cell.
    if(((theFrameInfo.time - cell.timestamp) + 1) * cell.priority > ((theFrameInfo.time - nextSearchCell.timestamp) + 1) * nextSearchCell.priority)
    {
      nextSearchCell = cell;
    }
  }
  const Vector2f positionCellToSearchNext = nextSearchCell.positionOnField;
  CROSS("module:BallSearchAreasProvider:fullGrid", positionCellToSearchNext.x(), positionCellToSearchNext.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);
  return positionCellToSearchNext;
}

std::list<SectorWheel::Sector> BallSearchAreasProvider::calculateObstacleSectors() const
{
  SectorWheel sectorWheel;
  sectorWheel.begin(theRobotPose.translation, maxDistanceToCell);
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f obstacleOnField = theRobotPose * obstacle.center;
    if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
      continue;
    const float obstacleWidth = (obstacle.left - obstacle.right).norm() + obstacleOffset;
    const float obstacleDistance = std::sqrt(std::max((obstacleOnField - theRobotPose.translation).squaredNorm() - sqr(obstacleWidth / 2.f), 1.f));
    const float obstacleRadius = std::atan(obstacleWidth / (2.f * obstacleDistance));
    const Angle obstacleDirection = (obstacleOnField - theRobotPose.translation).angle();
    sectorWheel.addSector(Rangea(Angle::normalize(obstacleDirection - obstacleRadius), Angle::normalize(obstacleDirection + obstacleRadius)), obstacleDistance, SectorWheel::Sector::obstacle);
  }
  return sectorWheel.finish();
}

void BallSearchAreasProvider::updateCells()
{
  const bool kickIn = theGameState.isKickIn();
  const bool goalKick = theGameState.isGoalKick();
  const bool cornerKick = theGameState.isCornerKick();

  // The sector wheel contains areas around the robot that are covered by an obstacle and therefore could not be updated within the update method.
  const std::list<SectorWheel::Sector> wheel =  calculateObstacleSectors();

  std::vector<Vector2f> p;
  //builds the polygon of visible field parts of the robot
  Projection::computeFieldOfViewInFieldCoordinates(theRobotPose, theCameraMatrix, theCameraInfo, theFieldDimensions, p);

  for(unsigned i = 0; i < cellCountX * cellCountY; i++)
  {
    BallSearchAreas::Cell& cell = grid[i];
    // This part of the code updates the priorities of the cells depending on the current game state.
    if((cornerKick
        && (Geometry::isPointInsideRectangle(theGameState.isForOpponentTeam() ? rectLeftOwnCorner : rectLeftOpponentCorner, cell.positionOnField)
            || Geometry::isPointInsideRectangle(theGameState.isForOpponentTeam() ? rectRightOwnCorner : rectRightOpponentCorner, cell.positionOnField)))
       || (goalKick
           && Geometry::isPointInsideRectangle(theGameState.isForOpponentTeam() ? rectOpponentGoalArea : rectOwnGoalArea, cell.positionOnField)))

    {
      // Use 8000 for priority to ensure the robot will prioritize the important cells for at least 8 sec.
      cell.priority = 8000;
    }
    else if(kickIn
            && (i < cellCountX
                || cellCountX * (cellCountY - 1) <= i))
    {
      const Vector2f left_corner(theBallDropInModel.outPosition.x() - cellWidth, theBallDropInModel.outPosition.y());
      const Vector2f right_corner(theBallDropInModel.outPosition.x() + cellWidth, theBallDropInModel.outPosition.y() + cellHeight);
      const Geometry::Rect out_pos_rect(left_corner, right_corner);
      if(Geometry::isPointInsideRectangle(out_pos_rect, cell.positionOnField))
      {
        // Use 8000 for priority to ensure the robot will prioritize the important cells for at least 8 sec.
        cell.priority = 8000;
      }
      else
      {
        // For the rest of the cells of the sideline the priority is 4000.
        cell.priority = 4000;
      }
    }
    else
    {
      cell.priority = 1;
    }

    if(cellInView(cell, p))
    {
      // For updating the timestamp it has to be checked if the cell is in sight of the robot and if its covered by an obstacle. Therefore a sector wheel is used.
      for(const auto& sector : wheel)
      {
        // If the cell is inside of a sector the timestamp will be updated. For the check if an obstacle covers the cell a polygon is calculated out of the sector using the distance to the sector boundaries.
        if(Geometry::isPointInsideArc(cell.positionOnField, theRobotPose.translation, sector.angleRange, sector.distance))
        {
          updateTimestamp(cell);
          break;
        }
      }
    }
  }
}
