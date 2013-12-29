#include <cstring>
#include <algorithm>
#include "GlobalFieldCoverageProvider.h"

#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/RotationMatrix.h"

MAKE_MODULE(GlobalFieldCoverageProvider, Modeling)

#define DRAW_CELL_COVERAGE(id, cell, cov) \
  const Vector2<int> coord = FieldCoverage::GridInterval::index2CellCoordinates(i); \
  const float x1 = theFieldDimensions.xPosOwnGroundline + (float) coord.x * cellLengths.x; \
  const float y1 = theFieldDimensions.yPosRightSideline + (float) coord.y * cellLengths.y; \
  const float x2 = x1 + cellLengths.x; \
  const float y2 = y1 + cellLengths.y; \
  const unsigned char alpha = 255 - cov; \
  RECTANGLE(id, x1+50, y1+50, x2-50, y2-50, 20, Drawings::ps_solid, ColorRGBA(0, 0, 255, alpha));

#define DRAW_CELL_COVERAGE_WITH_TEXT(id, cell, cov) \
  DRAW_CELL_COVERAGE(id, cell, cov); \
  const float y = theFieldDimensions.yPosRightSideline + (float) coord.y * cellLengths.y + 0.5f * cellLengths.y; \
  DRAWTEXT(id, x1, y, 100, ColorRGBA(255, 255, 255, 255), cov);

#define DRAW_FIELDCOVERAGE_ROBOT(id, robotId) \
  COMPLEX_DRAWING(id, \
  { \
    for(int i = 0; i < FieldCoverage::GridInterval::xSteps*FieldCoverage::GridInterval::ySteps; ++i) \
    { \
      unsigned char cov = coverage(robotId, i); \
      DRAW_CELL_COVERAGE(id, i, cov); \
    } \
  });

GlobalFieldCoverageProvider::GlobalFieldCoverageProvider()
  : xPosOwnGroundline(0.f), yPosRightSideline(0.f), throwInLineDistance(400.0f), throwInLineCellOffset(0)
{
  memset(fieldCoverageGrids, 0, sizeof(fieldCoverageGrids));
  for(int i = 0; i < FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps; ++i)
    cell2Region[i] = NULL;

  // Field dimensions of a cell.
  xPosOwnGroundline = theFieldDimensions.xPosOwnGroundline;
  yPosRightSideline = theFieldDimensions.yPosRightSideline;
  cellLengths.x = 2.0f * theFieldDimensions.xPosOpponentGroundline / static_cast<float>(FieldCoverage::GridInterval::xSteps);
  cellLengths.y = 2.0f * theFieldDimensions.yPosLeftSideline / static_cast<float>(FieldCoverage::GridInterval::ySteps);

  // Cell offset for throw (/drop) in
  throwInLineCellOffset = static_cast<int>(throwInLineDistance / cellLengths.y);
}

void GlobalFieldCoverageProvider::update(GlobalFieldCoverage& globalFieldCoverage)
{
  DECLARE_DEBUG_DRAWING3D("module:GlobalFieldCoverageProvider:globalFieldCoverage3D", "field");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:fieldCoverageRobot1", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:fieldCoverageRobot2", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:fieldCoverageRobot3", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:fieldCoverageRobot4", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:fieldCoverageRobot5", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:globalFieldCoverage", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:globalWorstCoveredCell", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:patrolRegionCenters", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:patrolRegions", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:patrolComponentCenters", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:patrolComponents", "drawingOnField");
  DRAW_FIELDCOVERAGE_ROBOT("module:GlobalFieldCoverageProvider:fieldCoverageRobot1", 1);
  DRAW_FIELDCOVERAGE_ROBOT("module:GlobalFieldCoverageProvider:fieldCoverageRobot2", 2);
  DRAW_FIELDCOVERAGE_ROBOT("module:GlobalFieldCoverageProvider:fieldCoverageRobot3", 3);
  DRAW_FIELDCOVERAGE_ROBOT("module:GlobalFieldCoverageProvider:fieldCoverageRobot4", 4);
  DRAW_FIELDCOVERAGE_ROBOT("module:GlobalFieldCoverageProvider:fieldCoverageRobot5", 5);
  DECLARE_PLOT("module:GlobalFieldCoverageProvider:coverageThreshold");
  DECLARE_PLOT("module:GlobalFieldCoverageProvider:patrolRegionIterations");

  // Update team mate field coverage grids
  updateFieldCoverageGrids();

  // Update global coverage grid
  if(theFieldCoverage.throwIn)
    ballThrowIn(globalFieldCoverage.grid);
  int worst = mergeGrids(globalFieldCoverage.grid);
  COMPLEX_DRAWING("module:GlobalFieldCoverageProvider:globalFieldCoverage",
  {
    for(size_t i = 0; i < FieldCoverage::GridInterval::xSteps* FieldCoverage::GridInterval::ySteps; i++)
    {
      const unsigned char coverage = globalFieldCoverage.grid.coverage(i, theFrameInfo.time);
      DRAW_CELL_COVERAGE_WITH_TEXT("module:GlobalFieldCoverageProvider:globalFieldCoverage", i, coverage);
    }
  });

  // Update index of globally worst covered cell.
  updateWorstCoveredCell(worst, globalFieldCoverage);
  COMPLEX_DRAWING("module:GlobalFieldCoverageProvider:globalWorstCoveredCell",
  {
    Vector2<int> coord = FieldCoverage::GridInterval::index2CellCoordinates(globalFieldCoverage.worstCoveredCellIndex);
    float x = theFieldDimensions.xPosOwnGroundline + (2 * (float) coord.x + 1) * cellLengths.x / 2.0f;
    float y = theFieldDimensions.yPosRightSideline + (2 * (float) coord.y + 1) * cellLengths.y / 2.0f;
    CIRCLE("module:GlobalFieldCoverageProvider:globalWorstCoveredCell", x, y, 100, 10, Drawings::ps_solid, ColorClasses::blue, Drawings::ps_solid, ColorClasses::blue);
  });

  // Find patrol regions
  const int numPatrol = initRegions();
  if(numPatrol)
  {
    STOP_TIME_ON_REQUEST_WITH_PLOT("GlobalFieldCoverageProvider:threshold", globalFieldCoverage.threshold = findThreshold(globalFieldCoverage););
    PLOT("module:GlobalFieldCoverageProvider:coverageThreshold", globalFieldCoverage.threshold);
    STOP_TIME_ON_REQUEST_WITH_PLOT("GlobalFieldCoverageProvider:regions", findRegions(globalFieldCoverage, numPatrol););
    STOP_TIME_ON_REQUEST_WITH_PLOT("GlobalFieldCoverageProvider:components", findLargestComponents(););
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
      if(!largestComponent[i].empty())
        calcCenterOfRegion(largestComponent[i], componentCenter[i]);

    if(largestComponent[theRobotInfo.number].empty())
      globalFieldCoverage.patrolTargetValid = false;
    else
    {
      globalFieldCoverage.patrolTargetValid = true;
      globalFieldCoverage.patrolTarget = componentCenter[theRobotInfo.number];
    }
  }
  else
  {
    globalFieldCoverage.patrolTargetValid = false;
  }

  COMPLEX_DRAWING("module:GlobalFieldCoverageProvider:patrolRegionCenters",
  {
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
    {
      Region& r = regions[i];
      if(r.robotNumber == -1)
        continue;
      if(r.cells)
      {
        CIRCLE("module:GlobalFieldCoverageProvider:patrolRegionCenters",
               r.mean.x, r.mean.y, 80, 10, Drawings::ps_solid, ColorClasses::red,
               Drawings::ps_solid, ColorRGBA(255, 192, 203)); // Pink
      }
      else
      {
        OUTPUT_WARNING("Region " << r.robotNumber << "has no cells.");
      }
    }
  });
  COMPLEX_DRAWING("module:GlobalFieldCoverageProvider:patrolRegions",
  {
    for(int x = 0; x < FieldCoverage::GridInterval::xSteps; ++x)
      for(int y = 0; y < FieldCoverage::GridInterval::ySteps; ++y)
      {
        const int c = x * FieldCoverage::GridInterval::ySteps + y;
        if(!cell2Region[c])
          continue;
        const float fx = xPosOwnGroundline + (static_cast<float>(x) + .5f) * cellLengths.x;
        const float fy = yPosRightSideline + (static_cast<float>(y) + .5f) * cellLengths.y;
        ColorClasses::Color cc = ColorClasses::none;
        Region* r = cell2Region[c];
        if(r->robotNumber == TeamMateData::player2)
          cc = ColorClasses::red;
        else if(r->robotNumber == TeamMateData::player3)
          cc = ColorClasses::blue;
        else if(r->robotNumber == TeamMateData::player4)
          cc = ColorClasses::yellow;
        else if(r->robotNumber == TeamMateData::player5)
          cc = ColorClasses::white;
        else
          ASSERT(false);
        CIRCLE("module:GlobalFieldCoverageProvider:patrolRegions",
               fx, fy, 100, 10, Drawings::ps_solid, cc, Drawings::ps_solid, cc);
      }
  });
  COMPLEX_DRAWING("module:GlobalFieldCoverageProvider:patrolComponents",
  {
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
    {
      const std::vector<int>& component = largestComponent[i];
      for(size_t c = 0; c < component.size(); ++c)
      {
        const float fx = xPosOwnGroundline
        + (static_cast<float>(component[c] / FieldCoverage::GridInterval::ySteps) + .5f) * cellLengths.x;
        const float fy = yPosRightSideline
        + (static_cast<float>(component[c] % FieldCoverage::GridInterval::ySteps) + .5f) * cellLengths.y;
        ColorClasses::Color cc = ColorClasses::none;
        if(i == TeamMateData::player2)
          cc = ColorClasses::red;
        else if(i == TeamMateData::player3)
          cc = ColorClasses::blue;
        else if(i == TeamMateData::player4)
          cc = ColorClasses::yellow;
        else if(i == TeamMateData::player5)
          cc = ColorClasses::white;
        else
          ASSERT(false); // All other robotnumbers should have empty components.
        CIRCLE("module:GlobalFieldCoverageProvider:patrolComponents",
               fx, fy, 100, 10, Drawings::ps_solid, cc, Drawings::ps_solid, cc);
      }
    }
  });
  COMPLEX_DRAWING("module:GlobalFieldCoverageProvider:patrolComponentCenters",
  {
    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
      if(!largestComponent[i].empty())
      {
        CIRCLE("module:GlobalFieldCoverageProvider:patrolComponentCenters",
               componentCenter[i].x, componentCenter[i].y, 80, 10, Drawings::ps_solid, ColorClasses::red,
               Drawings::ps_solid, ColorRGBA(255, 192, 203)); // Pink
      }
  });
  COMPLEX_DRAWING3D("module:GlobalFieldCoverageProvider:globalFieldCoverage3D", draw3D(globalFieldCoverage));
}

void GlobalFieldCoverageProvider::updateFieldCoverageGrids()
{
  for(int mate = TeamMateData::firstPlayer; mate < TeamMateData::numOfPlayers; ++mate)
    if(mate != theRobotInfo.number)
    {
      unsigned* fcgrid = fieldCoverageGrids[mate];
      const FieldCoverage::GridInterval& gridInterval = theTeamMateData.fieldCoverages[mate];
      // Avoid timestamps in the future
      const unsigned basetime = gridInterval.timestamp < theFrameInfo.time ? gridInterval.timestamp : theFrameInfo.time;
      const int intervalBase = gridInterval.interval * gridInterval.intervalSize;
      for(int i = 0; i < gridInterval.intervalSize; ++i)
      {
        const unsigned sub = gridInterval.tick * (gridInterval.maxCoverage - gridInterval.cells[i]);
        fcgrid[intervalBase + i] = sub >= basetime ? 0 : basetime - sub;
      }
    }
}

int GlobalFieldCoverageProvider::mergeGrids(GlobalFieldCoverage::Grid& mergedGrid)
{
  const size_t size = FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps;

  int worstIdx = -1;
  unsigned worstLastseen = 0;
  for(size_t i = 0; i < size; i++)
  {
    mergedGrid.cells[i] = std::max(mergedGrid.cells[i], theFieldCoverage.cells[i]);
    for(int j = TeamMateData::firstPlayer; j < TeamMateData::numOfPlayers; ++j)
      mergedGrid.cells[i] = std::max(mergedGrid.cells[i], fieldCoverageGrids[j][i]);

    if(worstIdx == -1 || worstLastseen > mergedGrid.cells[i])
    {
      worstIdx = i;
      worstLastseen = mergedGrid.cells[i];
    }
  }

  ASSERT(worstIdx != -1);
  return worstIdx;
}

void GlobalFieldCoverageProvider::updateWorstCoveredCell(int worstIdx, GlobalFieldCoverage& globalFieldCoverage)
{
  if(globalFieldCoverage.worstCoveredCellIndex == -1)
    globalFieldCoverage.worstCoveredCellIndex = worstIdx;

  if(globalFieldCoverage.worstCoveredCellIndex != worstIdx)
  {
    const unsigned char gfcCoverage = globalFieldCoverage.grid.coverage(globalFieldCoverage.worstCoveredCellIndex, theFrameInfo.time);
    if(gfcCoverage <= 1)
      return; // Cannot be much worse.

    const unsigned char currentCoverage = globalFieldCoverage.grid.coverage(worstIdx, theFrameInfo.time);
    if(currentCoverage < gfcCoverage - 1)
      globalFieldCoverage.worstCoveredCellIndex = worstIdx;
  }
}

void GlobalFieldCoverageProvider::calcCenterOfRegion(const std::vector<int>& region, Vector2<>& center) const
{
  center = Vector2<>(0.0f, 0.0f);
  std::vector<int>::const_iterator it = region.begin();
  std::vector<int>::const_iterator end = region.end();
  for(; it != end; ++it)
  {
    Vector2<int> coord = FieldCoverage::GridInterval::index2CellCoordinates(*it);
    center.x += xPosOwnGroundline + cellLengths.x * static_cast<float>(coord.x);
    center.y += yPosRightSideline + cellLengths.y * static_cast<float>(coord.y);
  }
  const float size = static_cast<float>(region.size());
  center.x /= size;
  center.y /= size;
  center.x += 0.5f * cellLengths.x;
  center.y += 0.5f * cellLengths.y;
}

unsigned char GlobalFieldCoverageProvider::coverage(int robotId, int cellIdx) const
{
  const int deltaT = theFrameInfo.getTimeSince(fieldCoverageGrids[robotId][cellIdx]);
  const int sub = deltaT <= 0 ? 0 : deltaT / FieldCoverage::GridInterval::tick;
  return static_cast<unsigned char>(sub >= FieldCoverage::GridInterval::maxCoverage ? 0 : FieldCoverage::GridInterval::maxCoverage - sub);
}

void GlobalFieldCoverageProvider::ballThrowIn(GlobalFieldCoverage::Grid& grid)
{
  for(int x = 0; x < FieldCoverage::GridInterval::xSteps; ++x)
  {
    const int right = FieldCoverage::GridInterval::cellCoordinates2Index(x, throwInLineCellOffset);
    const int left = FieldCoverage::GridInterval::cellCoordinates2Index(x, FieldCoverage::GridInterval::ySteps - 1 - throwInLineCellOffset);
    grid.setCoverage(right, theFrameInfo.time, 0);
    grid.setCoverage(left, theFrameInfo.time, 0);
  }
}

int GlobalFieldCoverageProvider::findThreshold(const GlobalFieldCoverage& coverage)
{
  // Build histogram
  int histogram[FieldCoverage::GridInterval::maxCoverage + 1];
  memset(histogram, 0, sizeof(histogram));
  for(int i = 0; i < FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps; ++i)
  {
    const int cov = coverage.grid.coverage(i, theFrameInfo.time);
    histogram[cov] += 1;
  }

  int sumCov2 = 0;
  int sumLow = 0; // Sum of coverage values <= the threshold
  int sumHigh = 0; // Sum of coverage values > the threshold
  int sumCovLow = 0; // Sum of coverage values <= the threshold weighted with its occurrence
  int sumCovHigh = 0; // Sum of coverage values > the threshold weighted with its occurrence

  // Initial threshold is 0.
  for(int cov = 0; cov <= FieldCoverage::GridInterval::maxCoverage; ++cov)
  {
    sumCov2 += histogram[cov] * cov * cov;
    sumCovHigh += histogram[cov] * cov;
    sumHigh += histogram[cov];
  }

  int covBestT = -1; // Best coverage threshold
  const float sumCov2F = static_cast<float>(sumCov2);
  float bestError = sumCov2F - static_cast<float>(sumCovHigh * sumCovHigh) / static_cast<float>(sumHigh);
  for(int t = 0; t <= FieldCoverage::GridInterval::maxCoverage; ++t)
  {
    sumHigh -= histogram[t];
    sumLow += histogram[t];
    sumCovLow += histogram[t] * t;
    sumCovHigh -= histogram[t] * t;
    if(sumLow > 0 && sumHigh > 0)
    {
      float error = sumCov2F - static_cast<float>(sumCovLow * sumCovLow) / static_cast<float>(sumLow)
                    - static_cast<float>(sumCovHigh * sumCovHigh) / static_cast<float>(sumHigh);
      if(error < bestError)
      {
        bestError = error;
        covBestT = t;
      }
    }
  }

  return covBestT;
}

int GlobalFieldCoverageProvider::initRegions()
{
  int patrolReady = 0;
  // Find players which are 'online' and not goalkeeper
  for(int p = TeamMateData::firstPlayer; p < TeamMateData::numOfPlayers; ++p)
  {
    if(theRobotInfo.number == p || !theTeamMateData.timeStamps[p])
      continue;
    if(theTeamMateData.isPenalized[p] || !theTeamMateData.hasGroundContact[p]
       || !theTeamMateData.isUpright[p] || theTeamMateData.robotPoses[p].deviation > 50.f
       || theTeamMateData.behaviorStatus[p].role == BehaviorStatus::keeper)
      removeFromPatrol(p);
    else
    {
      updatePatrol(theTeamMateData.robotPoses[p].translation, p);
      patrolReady += 1;
    }
  }
  if(theBehaviorControlOutput.behaviorStatus.role != BehaviorStatus::keeper && theRobotPose.deviation <= 50.f)
  {
    updatePatrol(theRobotPose.translation, theRobotInfo.number);
    patrolReady += 1;
  }
  else
    removeFromPatrol(theRobotInfo.number);

  return patrolReady;
}

void GlobalFieldCoverageProvider::updatePatrol(const Vector2<>& rpose, int robotNumber)
{
  ASSERT(robotNumber >= TeamMateData::firstPlayer
         && robotNumber < TeamMateData::numOfPlayers);
  regions[robotNumber].mean = rpose;
  regions[robotNumber].robotNumber = robotNumber;
}

void GlobalFieldCoverageProvider::removeFromPatrol(int robotNumber)
{
  ASSERT(robotNumber >= TeamMateData::firstPlayer
         && robotNumber < TeamMateData::numOfPlayers);
  if(regions[robotNumber].robotNumber == -1)
    return;
  regions[robotNumber].robotNumber = -1;
  for(int i = 0; i < FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps; ++i)
    if(cell2Region[i] == &regions[robotNumber])
      cell2Region[i] = NULL;
}

void GlobalFieldCoverageProvider::findRegions(const GlobalFieldCoverage& coverage, int numPatrol)
{
  int invalids;
  for(invalids = TeamMateData::firstPlayer; invalids < TeamMateData::numOfPlayers
      && regions[invalids].robotNumber == -1; ++invalids);
  if(invalids == TeamMateData::numOfPlayers)
    return; // No regions to assign cells to.

  // Find closest covered cell for each start cell.
  int iterations;
  for(iterations = 0; iterations < 10; ++iterations)
  {
    bool converged = true;
    for(int x = 0; x < FieldCoverage::GridInterval::xSteps; ++x)
      for(int y = 0; y < FieldCoverage::GridInterval::ySteps; ++y)
      {
        const int cellIdx = x * FieldCoverage::GridInterval::ySteps + y;
        if(coverage.grid.coverage(cellIdx, theFrameInfo.time) > coverage.threshold)
        {
          if(cell2Region[cellIdx])
          {
            Region* r = cell2Region[cellIdx];
            r->cells -= 1;
            r->acc.x -= x;
            r->acc.y -= y;
          }
          cell2Region[cellIdx] = NULL;
          continue; // Cell too well covered to be interesting
        }

        // Find closest region
        float minDistance = .0f;
        Region* bestRegion = NULL;
        Region* defenderRegion = NULL;
        for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
        {
          if(regions[i].robotNumber == -1)
            continue;
          // Don't assign cells in the opponent half of the field to the defender's region,
          // if the defender is in it's own half.
          const BehaviorStatus& bhstatus = theRobotInfo.number == regions[i].robotNumber ? theBehaviorControlOutput.behaviorStatus
                                       : theTeamMateData.behaviorStatus[regions[i].robotNumber];
          if(bhstatus.role == BehaviorStatus::defender && numPatrol > 1 && regions[i].mean.x < 0.f
             && x* cellLengths.x >= -xPosOwnGroundline)
          {
            defenderRegion = &regions[i];
            continue;
          }

          // Check whether region[i] is the closest region to cell (x,y)
          const float distance2 = regions[i].distance2(x, y, cellLengths, xPosOwnGroundline, yPosRightSideline);
          if(!bestRegion || distance2 < minDistance)
          {
            bestRegion  = &regions[i];
            minDistance = distance2;
          }
        }
        ASSERT(bestRegion || defenderRegion);
        if(!bestRegion)
          bestRegion = defenderRegion;
        // Update cluster if necessary.
        if(cell2Region[cellIdx] != bestRegion)
        {
          bestRegion->acc.x += x;
          bestRegion->acc.y += y;
          bestRegion->cells += 1;
          if(cell2Region[cellIdx])
          {
            Region* prev = cell2Region[cellIdx];
            prev->acc.x -= x;
            prev->acc.y -= y;
            prev->cells -= 1;
          }
          cell2Region[cellIdx] = bestRegion;
          converged = false;
        }
      }
    if(converged)
      break;

    for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
      regions[i].adjustMean(cellLengths, xPosOwnGroundline, yPosRightSideline);
  }
  PLOT("module:GlobalFieldCoverageProvider:patrolRegionIterations", iterations);
}

void GlobalFieldCoverageProvider::field2Cell(Vector2<int>& cell, const Vector2<>& translation) const
{
  cell.x = static_cast<int>(translation.x - theFieldDimensions.xPosOwnGroundline);
  cell.x /= static_cast<int>(cellLengths.x);
  cell.y = static_cast<int>(translation.y - theFieldDimensions.yPosRightSideline);
  cell.y /= static_cast<int>(cellLengths.y);
}

float GlobalFieldCoverageProvider::Region::distance2(const Region& other) const
{
  return (mean - other.mean).sqr();
}

float GlobalFieldCoverageProvider::Region::distance2(int cx, int cy, const Vector2<>& cellLengths,
    float xPosOwnGroundline, float yPosRightSideline) const
{
  float x = xPosOwnGroundline;
  x += cellLengths.x * static_cast<float>(cx);
  float y = yPosRightSideline;
  y += cellLengths.y * static_cast<float>(cy);
  const float dx = mean.x - x;
  const float dy = mean.y - y;
  return dx * dx + dy * dy;
}

void GlobalFieldCoverageProvider::Region::adjustMean(const Vector2<>& cellLengths, float xPosOwnGroundline, float yPosRightSideline)
{
  if(!cells)
    return; // No cells => No mean among the cells.

  const float size = static_cast<float>(cells);
  mean.x = xPosOwnGroundline;
  mean.x += static_cast<float>(acc.x) / size * cellLengths.x + .5f * cellLengths.x;
  mean.y = yPosRightSideline;
  mean.y += static_cast<float>(acc.y) / size * cellLengths.y + .5f * cellLengths.y;
}

void GlobalFieldCoverageProvider::findLargestComponents()
{
  // Clear old largest components
  for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
    largestComponent[i].clear();

  // Find new largest components
  const int xSteps = FieldCoverage::GridInterval::xSteps;
  const int ySteps = FieldCoverage::GridInterval::ySteps;
  bool marked[xSteps * ySteps] = { false };
  for(int x = 0; x < xSteps; ++x)
    for(int y = 0; y < ySteps; ++y)
    {
      const Region* const region = cell2Region[x * ySteps + y];
      if(!marked[x * ySteps + y] && region)
      {
        std::vector<int> component;
        findComponent(x, y, marked, component);
        // Component found
        if(largestComponent[region->robotNumber].size() < component.size())
          largestComponent[region->robotNumber].swap(component);
      }
    }
}

void GlobalFieldCoverageProvider::findComponent(int x, int y, bool marked[], std::vector<int>& component)
{
  const int ySteps = FieldCoverage::GridInterval::ySteps;
  const int xSteps = FieldCoverage::GridInterval::xSteps;
  const int cidx = x * ySteps + y;
  if(marked[cidx])
    return; // Alerady dealt with cell (x, y)

  const Region* const region = cell2Region[cidx];  // Region this component is part of.
  if(!region)
    return; // Not part of any region, cannot be part of a component

  std::stack<int> stack;
  stack.push(cidx);
  marked[cidx] = true;
  while(!stack.empty())
  {
    int cell = stack.top();
    stack.pop();
    component.push_back(cell);
    if((cell % ySteps != 0) && (cell2Region[cell - 1] == region) && !marked[cell - 1]) // Not at the right border of the grid
    {
      stack.push(cell - 1);
      marked[cell - 1] = true;
    }
    if((cell % ySteps != ySteps - 1) && (cell2Region[cell + 1] == region) && !marked[cell + 1]) // Not at the left border
    {
      stack.push(cell + 1);
      marked[cell + 1] = true;
    }
    if((cell >= ySteps) && (cell2Region[cell - ySteps] == region) && !marked[cell - ySteps]) // Not at the bottom row
    {
      stack.push(cell - ySteps);
      marked[cell - ySteps] = true;
    }
    if((cell < ySteps * (xSteps - 1)) && (cell2Region[cell + ySteps] == region) && !marked[cell + ySteps]) // Not at the top row
    {
      stack.push(cell + ySteps);
      marked[cell + ySteps] = true;
    }
  }
}

void GlobalFieldCoverageProvider::draw3D(const GlobalFieldCoverage& globalFieldCoverage) const
{
  const float spacing = 0.02f;
  const float robotSpacing = 3.0f * spacing;
  const float height = 60.0f;
  const float robotHeight = 1.3f * height;
  const float targetArrowHeight = 2.0f * robotHeight;
  const float targetArrowLength = 250.0f;
  const float targetArrowRadius = 10.0f;
  const float targetArrowTipRadius = 5.0f * targetArrowRadius;
  const float targetArrowTipLength = 0.1f * targetArrowLength;
  RotationMatrix fieldRotation; // Identity by default
  /* for(int y = 0; y < FieldCoverage::GridInterval::ySteps; ++y) */
  for(int x = 0; x < FieldCoverage::GridInterval::xSteps; ++x)
    for(int y = 0; y < FieldCoverage::GridInterval::ySteps; ++y)
    {
      const float fx = static_cast<float>(x);
      const float fy = static_cast<float>(y);
      const int cell = x * FieldCoverage::GridInterval::ySteps + y;
      const float bottom = xPosOwnGroundline + (fx + spacing) * cellLengths.x;
      const float top = xPosOwnGroundline + (fx + 1 - spacing) * cellLengths.x;
      const float right = yPosRightSideline + (fy + spacing) * cellLengths.y;
      const float left = yPosRightSideline + (fy + 1 - spacing) * cellLengths.y;
      const Vector2<> center(xPosOwnGroundline + (fx + 0.5f) * cellLengths.x,
                             yPosRightSideline + (fy + 0.5f) * cellLengths.y);

      // Display global grid.
      const unsigned char value = globalFieldCoverage.grid.coverage(cell, theFrameInfo.time);
      const Vector3<> bottomRight(bottom, right, height);
      const Vector3<> topRight(top, right, height);
      const Vector3<> topLeft(top, left, height);
      const Vector3<> bottomLeft(bottom, left, height);
      QUAD3D("module:GlobalFieldCoverageProvider:globalFieldCoverage3D",
             fieldRotation * bottomRight, fieldRotation * topRight, fieldRotation * topLeft, fieldRotation * bottomLeft,
             ColorRGBA(255, 0, 0, 255 - value));

      // Display grids of other robots, only works with the current player numbering.
      ASSERT(TeamMateData::firstPlayer == 1);
      ASSERT((TeamMateData::numOfPlayers - TeamMateData::firstPlayer) == 4);
      for(int i = 0; i < 2 ; ++i)
        for(int j = 0; j < 2; ++j)
        {
          const int robotId = 1 + 2 * i + j;
          const float robotRight = (j ? right : center.y) + robotSpacing * cellLengths.y;
          const float robotLeft = (j ? center.y : left) - robotSpacing * cellLengths.y;
          const float robotBottom = (i ? bottom : center.x) + robotSpacing * cellLengths.x;
          const float robotTop = (i ? center.x : top) - robotSpacing * cellLengths.x;

          const Vector3<> robotBottomRight(robotBottom, robotRight, robotHeight);
          const Vector3<> robotTopRight(robotTop, robotRight, robotHeight);
          const Vector3<> robotTopLeft(robotTop, robotLeft, robotHeight);
          const Vector3<> robotBottomLeft(robotBottom, robotLeft, robotHeight);

          const unsigned char robotValue = (robotId != theRobotInfo.number) ? coverage(robotId, cell) : theFieldCoverage.coverage(cell, theFrameInfo.time);;
          QUAD3D("module:GlobalFieldCoverageProvider:globalFieldCoverage3D",
                 fieldRotation * robotBottomRight, fieldRotation * robotTopRight, fieldRotation * robotTopLeft, fieldRotation * robotBottomLeft,
                 ColorRGBA(255 - robotValue, 0, robotValue, 255));
        }

      // Draw a pink arrow for every patrol target over the field pointing towards the target on ground.
      for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
        if(!largestComponent[i].empty())
        {
          const Vector2<>& ptarget = componentCenter[i];
          const Vector3<> to(ptarget.x, ptarget.y, targetArrowHeight);
          const Vector3<> from(ptarget.x, ptarget.y, targetArrowHeight + targetArrowLength);
          CYLINDERARROW3D("module:GlobalFieldCoverageProvider:globalFieldCoverage3D", fieldRotation * from, fieldRotation * to, targetArrowRadius, targetArrowTipLength, targetArrowTipRadius, ColorRGBA(255, 192, 203));
        }
    }
}
