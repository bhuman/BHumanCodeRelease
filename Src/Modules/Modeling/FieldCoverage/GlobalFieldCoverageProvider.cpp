/**
* @file GlobalFieldCoverageProvider.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "GlobalFieldCoverageProvider.h"
#include <string>

MAKE_MODULE(GlobalFieldCoverageProvider, modeling);

GlobalFieldCoverageProvider::GlobalFieldCoverageProvider()
{}

void GlobalFieldCoverageProvider::update(GlobalFieldCoverage& globalFieldCoverage)
{
  BH_TRACE;

  if(!initDone)
    init(globalFieldCoverage);

  globalFieldCoverage.newDataThisFrame = false;

  auto addLine = [&](const FieldCoverage::GridLine& line)
  {
    const int base = line.y * FieldCoverage::numOfCellsX;

    BH_TRACE_MSG(std::to_string(line.y).c_str());

    for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
    {
      if(line.timestamps[x] > globalFieldCoverage.grid[base + x].timestamp)
      {
        globalFieldCoverage.grid[base + x].timestamp = line.timestamps[x];
        globalFieldCoverage.grid[base + x].value = line.values[x];
        globalFieldCoverage.newDataThisFrame = true;
      }
    }
  };

  BH_TRACE;
  for(int y = 0; y < FieldCoverage::numOfCellsY; ++y)
    addLine(theFieldCoverage[y]);

  BH_TRACE;
  for(const auto teammate : theTeammateData.teammates)
    addLine(teammate.fieldCoverageLine);

  BH_TRACE;
}

void GlobalFieldCoverageProvider::init(GlobalFieldCoverage& globalFieldCoverage)
{
  BH_TRACE;

  initDone = true;

  globalFieldCoverage.grid.reserve(FieldCoverage::numOfCellsY * FieldCoverage::numOfCellsX);

  const float cellLengthX = theFieldDimensions.xPosOpponentGroundline / FieldCoverage::numOfCellsX * 2;
  const float cellLengthY = theFieldDimensions.yPosLeftSideline / FieldCoverage::numOfCellsY * 2;

  float positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
  float positionOnFieldY = theFieldDimensions.yPosRightSideline + cellLengthY / 2.f;

  const unsigned time = std::max(10000u, theFrameInfo.time);
  for(int y = 0; y < FieldCoverage::numOfCellsY; ++y)
  {
    for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
    {
      globalFieldCoverage.grid.emplace_back(time, time, positionOnFieldX, positionOnFieldY, cellLengthX, cellLengthY);
      positionOnFieldX += cellLengthX;
    }
    positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
    positionOnFieldY += cellLengthY;
  }
  BH_TRACE;
}