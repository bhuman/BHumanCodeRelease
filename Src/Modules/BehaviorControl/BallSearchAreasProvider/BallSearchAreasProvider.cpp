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
  cellCountX = static_cast<int>(std::abs(theFieldDimensions.xPosOwnGroundLine) * 2 / cellSize + 1);
  cellCountY = static_cast<int>(std::abs(theFieldDimensions.yPosRightSideline * 2) / cellSize + 1);
  colorVector.reserve(cellCountX * cellCountY);
  for(unsigned y = 0; y < cellCountY; y++)
  {
    for(unsigned x = 0; x < cellCountX ; x++)
    {
      const Vector2f positionOfCell(theFieldDimensions.xPosOwnGroundLine + x * cellSize, theFieldDimensions.yPosRightSideline + y * cellSize);
      grid.emplace_back(y * cellCountX + x, 0, 0, positionOfCell);
    }
  }
  ASSERT(grid.size() != 0);
}

void BallSearchAreasProvider::update(BallSearchAreas& theBallSearchAreas)
{
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:id", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:time", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:nextSearch", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:heatmap", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:fullGrid", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:priority", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallSearchAreasProvider:cornerArea", "drawingOnField");

  ASSERT(grid.size() != 0);
  reset();
  updateCells();
  draw();
  theBallSearchAreas.cellToSearchNext = [this](const Agent& agent) { auto vec = getVoronoiGrid(agent);  return positionCellToSearchNext(vec); };
  theBallSearchAreas.filterCellsToSearch = [this](const Agent& agent) { return getVoronoiGrid(agent); };
}

void BallSearchAreasProvider::draw()
{
  // ID - center, time - bottom left, priority - top left
  colorVector.clear();
  for(unsigned i = 0; i < cellCountX * cellCountY; i++)
  {
    DRAW_TEXT("module:BallSearchAreasProvider:id", grid[i].positionOnField.x(), grid[i].positionOnField.y(), 100, ColorRGBA::red, grid[i].id);

    const int timeLastSeen = theFrameInfo.time - grid[i].timestamp;
    const float max = 6000.f;
    const unsigned char alpha = static_cast<unsigned char>((max - timeLastSeen) / max * heatmapAlpha);
    // Interpolation between black(> max) - yellow(0).
    const ColorRGBA cellColor = timeLastSeen > max ? ColorRGBA(0, 0, 0, heatmapAlpha) : ColorRGBA(255, 255, 0, alpha).blend(ColorRGBA(0, 0, 0, heatmapAlpha - alpha));
    colorVector.push_back(cellColor);

    //Draw time
    DRAW_TEXT("module:BallSearchAreasProvider:heatmap", grid[i].positionOnField.x() - cellSize / 2, grid[i].positionOnField.y() - cellSize / 2, 100, ColorRGBA::white, timeLastSeen * grid[i].priority);
    DRAW_TEXT("module:BallSearchAreasProvider:fullGrid", grid[i].positionOnField.x() - cellSize / 2, grid[i].positionOnField.y() - cellSize / 2, 100, ColorRGBA::white, timeLastSeen * grid[i].priority);
    DRAW_TEXT("module:BallSearchAreasProvider:time", grid[i].positionOnField.x() - cellSize / 2, grid[i].positionOnField.y() - cellSize / 2, 100, ColorRGBA::white, timeLastSeen * grid[i].priority);

    //Draw Priority
    DRAW_TEXT("module:BallSearchAreasProvider:fullGrid", grid[i].positionOnField.x() - cellSize / 2, grid[i].positionOnField.y() + cellSize / 2, 100, ColorRGBA::black, grid[i].priority);
    DRAW_TEXT("module:BallSearchAreasProvider:priority", grid[i].positionOnField.x() - cellSize / 2, grid[i].positionOnField.y() + cellSize / 2, 100, ColorRGBA::black, grid[i].priority);
  }
  COMPLEX_DRAWING("module:BallSearchAreasProvider:fullGrid")
  {
    const Vector2f startPosition = theRobotPose.translation;
    DRAW_SECTOR_WHEEL("module:BallSearchAreasProvider:fullGrid", calculateObstacleSectors(), startPosition);
  }
  GRID_RGBA("module:BallSearchAreasProvider:heatmap", 0, 0, cellSize, cellCountX, cellCountY, colorVector.data());
  RECTANGLE("module:BallSearchAreasProvider:cornerArea", rectLeftOpponentCorner.a.x(), rectLeftOpponentCorner.a.y(), rectLeftOpponentCorner.b.x(), rectLeftOpponentCorner.b.y(), 100, Drawings::solidPen, ColorRGBA::red);
  RECTANGLE("module:BallSearchAreasProvider:cornerArea", rectRightOpponentCorner.a.x(), rectRightOpponentCorner.a.y(), rectRightOpponentCorner.b.x(), rectRightOpponentCorner.b.y(), 100, Drawings::solidPen, ColorRGBA::red);
  RECTANGLE("module:BallSearchAreasProvider:cornerArea", rectLeftOwnCorner.a.x(), rectLeftOwnCorner.a.y(), rectLeftOwnCorner.b.x(), rectLeftOwnCorner.b.y(), 100, Drawings::solidPen, ColorRGBA::blue);
  RECTANGLE("module:BallSearchAreasProvider:cornerArea", rectRightOwnCorner.a.x(), rectRightOwnCorner.a.y(), rectRightOwnCorner.b.x(), rectRightOwnCorner.b.y(), 100, Drawings::solidPen, ColorRGBA::blue);
}

void BallSearchAreasProvider::updateTimestamp(BallSearchAreas::Cell& cell)
{
  cell.timestamp = theFrameInfo.time;
}

bool BallSearchAreasProvider::cellInView(const BallSearchAreas::Cell& cell) const
{
  Vector2f pointInImage;
  if(Transformation::robotToImage(theRobotPose.inverse() * cell.positionOnField, theCameraMatrix, theCameraInfo, pointInImage) && returnDistanceToCell(cell) < maxDistanceToCell)
  {
    return (pointInImage.x() >= 0 &&
            pointInImage.x() <= theCameraInfo.width &&
            pointInImage.y() >= 0 &&
            pointInImage.y() <= theCameraInfo.height);
  }
  return false;
}

std::vector<BallSearchAreas::Cell> BallSearchAreasProvider::getVoronoiGrid(const Agent& agent) const
{
  std::vector<BallSearchAreas::Cell> voronoiGrid;

  for(const auto& cell : grid)
  {
    if(Geometry::isPointInsidePolygon(cell.positionOnField, agent.baseArea))
    {
      voronoiGrid.emplace_back(cell);
    }
  }
  return voronoiGrid;
}

float BallSearchAreasProvider::returnDistanceToCell(const BallSearchAreas::Cell& cell) const
{
  return (cell.positionOnField - theRobotPose.translation).norm();
}

void BallSearchAreasProvider::reset()
{
  if(theGameState.timeWhenStateStarted == theFrameInfo.time && (theGameState.isCornerKick() || theGameState.isKickIn() || theGameState.isGoalKick() || theGameState.isKickOff()))
  {
    for(auto& cell : grid)
    {
      cell.timestamp = theFrameInfo.getTimeSince(theFrameInfo.time);
    }
  }
}

const Vector2f BallSearchAreasProvider::positionCellToSearchNext(std::vector<BallSearchAreas::Cell>& gridToSearch) const
{
  ASSERT(gridToSearch.size() != 0);
  BallSearchAreas::Cell nextSearchCell = gridToSearch[0];

  for(auto& cell : gridToSearch)
  {
    // Saves the cell with the highest search score, which is calculatet by: If the difference between the timestamp of the cell and the timestamp of the cameraframe multiplied by the priority (evaluation of how long a cell has not been seen multiplied by the importance of the cell in the current gamestate) is higher than the stored value, replace the stored cell.
    if(((theFrameInfo.time - cell.timestamp) + 1) * cell.priority > ((theFrameInfo.time - nextSearchCell.timestamp) + 1) * nextSearchCell.priority)
    {
      nextSearchCell = cell;
    }
  }
  const Vector2f positionCellToSearchNext = nextSearchCell.positionOnField;
  CROSS("module:BallSearchAreasProvider:nextSearch", positionCellToSearchNext.x(), positionCellToSearchNext.y(), 100, 20, Drawings::solidPen, ColorRGBA::violet);
  return positionCellToSearchNext;
}

std::list<SectorWheel::Sector> BallSearchAreasProvider::calculateObstacleSectors() const
{
  SectorWheel sectorWheel;
  sectorWheel.begin(theRobotPose.translation);
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
  {
    const Vector2f obstacleOnField = theRobotPose * obstacle.center;
    if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGroundLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
      continue;
    const float obstacleWidth = (obstacle.left - obstacle.right).norm() + obstacleOffset;
    const float obstacleDistance = std::sqrt(std::max((obstacleOnField - theRobotPose.translation).squaredNorm() - sqr(obstacleWidth / 2.f), 1.f));
    const float obstacleRadius = std::atan(obstacleWidth / (2.f * obstacleDistance));
    const Angle obstacleDirection = (obstacleOnField - theRobotPose.translation).angle();
    sectorWheel.addSector(Rangea(Angle::normalize(obstacleDirection - obstacleRadius), Angle::normalize(obstacleDirection + obstacleRadius)), obstacleDistance, SectorWheel::Sector::obstacle);
    sectorWheel.addSector(Rangea(Angle::normalize(obstacleDirection + obstacleRadius), Angle::normalize(obstacleDirection - obstacleRadius)), maxDistanceToCell, SectorWheel::Sector::free);
  }
  return sectorWheel.finish();
}

void BallSearchAreasProvider::updateCells()
{
  const bool kickIn = theGameState.isKickIn();
  const bool goalKick = theGameState.isGoalKick();
  const bool cornerForOpponents = theGameState.isCornerKick() && theGameState.isForOpponentTeam();
  const bool cornerForOwn = theGameState.isCornerKick() && theGameState.isForOwnTeam();

  Vector2f ball_out_position = theBallDropInModel.outPosition;
  // The sector wheel contains areas around the robot that are covered by an obstacle and therefore could not be updated within the update method.
  const std::list<SectorWheel::Sector> wheel =  calculateObstacleSectors();
  if(!grid.empty())
  {
    for(auto& cell : grid)
    {
      //This part of the code updates the priorities of the cells depending on the current game state
      if((cornerForOpponents
          && (Geometry::isPointInsideRectangle(rectLeftOwnCorner, cell.positionOnField)
              || Geometry::isPointInsideRectangle(rectRightOwnCorner, cell.positionOnField)))
         || (cornerForOwn
             && (Geometry::isPointInsideRectangle(rectLeftOpponentCorner, cell.positionOnField)
                 || Geometry::isPointInsideRectangle(rectRightOpponentCorner, cell.positionOnField)))
         || (goalKick
             && Geometry::isPointInsideRectangle(rectOwnGoalArea, cell.positionOnField)))
      {
        cell.priority = 5;
      }
      else if(kickIn
              && (cell.id < cellCountX
                  || cellCountX * (cellCountY - 1) <= cell.id))
      {
        const Vector2f left_corner(theBallDropInModel.outPosition.x() - cellSize, theBallDropInModel.outPosition.y());
        const Vector2f right_corner(theBallDropInModel.outPosition.x() + cellSize, theBallDropInModel.outPosition.y() + cellSize);
        const Geometry::Rect out_pos_rect(left_corner, right_corner);
        if(Geometry::isPointInsideRectangle(out_pos_rect, cell.positionOnField))
        {
          cell.priority = 5;
        }
        else
        {
          cell.priority = 3;
        }
      }
      else
      {
        cell.priority = 1;
      }
      if(cellInView(cell))
      {
        // For updating the timestamp it has to be checked if the cell is in sight of the robot and if its covered by an obstacle. Therefore a sector wheel is used.
        for(const auto& sector : wheel)
        {
          // If the cell is inside of a sector the timestamp will be updated. For the check if an obstacle coveres the cell a polygon is calculated out of the sector using the distance to the sector boundaries.*/
          if(Geometry::isPointInsideArc(cell.positionOnField, theRobotPose.translation, sector.angleRange, sector.distance))
          {
            updateTimestamp(cell);
            break;
          }
        }
      }
    }
  }
}
